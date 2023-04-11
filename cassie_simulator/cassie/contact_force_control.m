function tau = contact_force_control(t, s, model)
    % generalized coordinates and velocities
    q = s(1:model.n);
    dq = s(model.n+1:2*model.n);

    % model parameters
    m = model.M;            % robot mass [kg]
    mu = 0.8;               % friction coefficient
    g = [0; 0; 9.81];       % acceleration due to gravity [m/s^2]

    % COM positions and body orientations
    rc_d = [-0.0103; 0; 0.8894];
    drc_d = zeros(3,1);
    R_d = eye(3);
    Omega_d = zeros(3,1);

    [rc, drc] = computeComPosVel(q, dq, model);
    Xb = bodypos(model, model.idx.torso, q);
    Rb = Xb(1:3,1:3)';
    % Rb = euler_to_rotm([yaw pitch roll])';
    Jdq = bodyJac(model, model.idx.torso, q)*dq;
    Omega = Jdq(1:3);

    % desired vel = -current vel, to maintain equilibrium
    Omega_d = -Omega;
    drc_d = -drc;

    %% 1) stabilizing wrench at COM (F_d)
    % proportional and differential gain
    kh = 900; kv = 3000; Kp = diag([kh kh kv]);
    dh = sqrt(m*kh)*2*0.8; dv = sqrt(m*kv)*2*0.2; Kd = diag([dh dh dv]);
    f_d = -Kp*(rc - rc_d) - Kd*(drc - drc_d) + m*g;     % + m*ddrc_d;

    Kr = 100*eye(3); Dr = 50*eye(3);    % rotational stiffness and damping
    Rdb = R_d'*Rb; Rwb = Rb; Rwb = eye(3);
    quat = rotm_to_quaternion(Rdb); delta = quat(1); epsilon = quat(2:4);
    tau_r = -2*(delta*eye(3) + hat(epsilon))*Kr*epsilon;
    % error definition 1: quaternion
    tau_d = Rwb*(tau_r - Dr*(Omega - Omega_d));
    % error definition 2: rotation matrix
    % tau_d = -Kr*error(Rb,R_d) - Dr*(Omega - Rb'*R_d*Omega_d); % + I*dOmega_d

    % correct the behavior for high yaw perturbation
    % can't differentiate between high Fz and high yaw?, use roll instead of dq(3)
    if t < 0.1 && abs(dq(3)) > 0.05  && abs(dq(3)) < 0.55 && norm(dq(4:6)) > 0.05
        tau_d = tau_d*7.5;
    end

    % correct the behavior for high x, y, z perturbation

    F_d = [f_d; tau_d];

    %% 2) contact forces to produce desired wrench (fc)
    Gc = contact_grasp_map(s, model, rc);

    n = 10;                             % friction cone pyramid, # sides           
    nj_mat = nj_matrix(n, mu);          % friction cone normal vectors

    A = zeros(4,12); A(sub2ind(size(A),1:4,3:3:12)) = -1;
    b = zeros(4,1);                     % unilateral constraint fi_z >= 0
    A = [A; -nj_mat];
    b = [b; zeros(n*4,1)];              % friction cone constraint
    Aeq = []; beq = [];
    % instead of using equality constraint (Aeq*x = beq; Gc*fc = F_d),
    % add ||F_d - Gc*fc||^2 to the cost function with weights prioritizing:
    % getting correct f_d >> getting correct tau_d >>  minimizing ||fc||^2 
    a1 = 10; a2 = 1; a3 = 0.01;         % a1 >> a2 >> a3
    A1 = [eye(3) zeros(3)];             % extract forces:  [I 0] ||F_d - Gc*fc||^2
    A2 = [zeros(3) eye(3)];             % extract torques: [0 I] ||F_d - Gc*fc||^2
    Gc1 = A1*Gc; F_d1 = A1*F_d; Gc2 = A2*Gc; F_d2 = A2*F_d; 
    H = Gc1'*Gc1*2*a1 + Gc2'*Gc2*2*a2 + a3*2*eye(12);
    f = (-2*a1*F_d1'*Gc1)' + (-2*a2*F_d2'*Gc2)' + a3*zeros(12,1);
    lb = []; ub = []; x0 = []; options =  optimset('Display','off');
    fc = quadprog(H,f,A,b,Aeq,beq,...
        lb,ub,x0,options);              % min 0.5*x'*H*x + f'*x
    % fc = Gc\F_d;

    %% 3) motor torques to produce desired contact forces (tau)
    [J1f, J1b, J2f, J2b] = computeFootJacobians(s, model);
    Jk = {J1f J1b J2f J2b};

    tau = zeros(10,1);
    for i = 1:4
        ind = i + 2*(i-1);
        Gamma_16 = Jk{i}' * [zeros(3,1); fc(ind:ind+2)];
        tau = tau + -Gamma_16(7:end);
    end
end

% computes body orientation from Euler angles, euler = [yaw pitch roll]
function R = euler_to_rotm(euler)
    yaw = euler(1); pitch = euler(2); roll = euler(3);
    Rroll  = @(r) [1 0 0; 0 cos(r) sin(r); 0 -sin(r) cos(r)];
    Rpitch = @(p) [cos(p) 0 -sin(p); 0 1 0; sin(p) 0 cos(p)];
    Ryaw   = @(y) [cos(y) sin(y) 0; -sin(y) cos(y) 0; 0 0 1];
    R = Rroll(roll)*Rpitch(pitch)*Ryaw(yaw);
end

% computes quaternion from Euler angles, euler = [yaw pitch roll]
function quat = euler_to_quaternion(euler)
    yaw = euler(1); pitch = euler(2); roll = euler(3);
    y = yaw; p = pitch; r = roll;   % rotation sequence: yaw-pitch-roll

    qx = sin(r/2)*cos(p/2)*cos(y/2) - cos(r/2)*sin(p/2)*sin(y/2);
    qy = cos(r/2)*sin(p/2)*cos(y/2) + sin(r/2)*cos(p/2)*sin(y/2);
    qz = cos(r/2)*cos(p/2)*sin(y/2) - sin(r/2)*sin(p/2)*cos(y/2);
    qw = cos(r/2)*cos(p/2)*cos(y/2) + sin(r/2)*sin(p/2)*sin(y/2);

    % qw = scalar component, [qx; qy; qz] = vector component
    quat = [qw; qx; qy; qz];        % quat = w + xi + yj + zk
end

% computes quaternion from rotation matrix
function quat = rotm_to_quaternion(rotm)
    m00 = rotm(1,1); m01 = rotm(1,2); m02 = rotm(1,3);
    m10 = rotm(2,1); m11 = rotm(2,2); m12 = rotm(2,3);
    m20 = rotm(3,1); m21 = rotm(3,2); m22 = rotm(3,3);
    tr = m00 + m11 + m22;

    if (tr > 0) 
      S = sqrt(tr + 1) * 2;                 % S = 4*qw 
      qw = 0.25*S;
      qx = (m21 - m12) / S;
      qy = (m02 - m20) / S; 
      qz = (m10 - m01) / S; 
    elseif ((m00 > m11) && (m00 > m22))
      S = sqrt(1 + m00 - m11 - m22) * 2;    % S = 4*qx 
      qw = (m21 - m12) / S;
      qx = 0.25*S;
      qy = (m01 + m10) / S; 
      qz = (m02 + m20) / S; 
    elseif (m11 > m22)
      S = sqrt(1 + m11 - m00 - m22) * 2;    % S = 4*qy
      qw = (m02 - m20) / S;
      qx = (m01 + m10) / S; 
      qy = 0.25*S;
      qz = (m12 + m21) / S; 
    else
      S = sqrt(1 + m22 - m00 - m11) * 2;    % S = 4*qz
      qw = (m10 - m01) / S;
      qx = (m02 + m20) / S;
      qy = (m12 + m21) / S;
      qz = 0.25*S;
    end

    % qw = scalar component, [qx; qy; qz] = vector component
    quat = [qw; qx; qy; qz];                % quat = w + xi + yj + zk
end

% computes vector error between rotation matrices
function err = error(R, Rdes)
    err = 0.5*vee(Rdes'*R - R'*Rdes);
    function x = vee(x_hat)
        x = skew(x_hat);
    end
end

% computes contact grasp map
function Gc = contact_grasp_map(s, model, rc)
    [p1, p2, p3, p4] = computeFootPositions(s, model);
    p = [p1 p2 p3 p4] - rc; % positions of contact points relative to COM

    Gc = zeros(6,3*4);
    Rpi = eye(3);
    for i = 1:4
        ind = i + 2*(i-1);
        Gc(1:3,ind:ind+2) = Rpi;
        Gc(4:6,ind:ind+2) = hat(p(:,i))*Rpi;
    end
end

% computes hat map of vector
function x_hat = hat(x)
    % x_hat = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    x_hat = skew(x);
end

% computes normal vectors for friction cone pyramid approximation
function nj = pyramid(n, mu)
    % mu                % coefficient of friction
    % n                 % number of sides of polygon

    % define points around base of pyramid
    angles = linspace(0,2*pi,n+1);
    angles = angles(1:end-1);
    x = mu*cos(angles); y = mu*sin(angles); z = ones(1,n);
    points = [x; y; z];
    origin = [0; 0; 0];
    
    % find normal vector to plane from 3 points
    nj = zeros(3,n);    % normal vectors returned as 3 x n matrix
    for i = 1:n
        v1 = points(:,i) - origin;
        v2 = points(:,mod(i,n)+1) - origin;
        nj(:,i) = cross(v1,v2);
        nj(:,i) = nj(:,i)/norm(nj(:,i));
    end
end

% returns inequality matrix for friction cone constraint to use in quadprog()
function nj_mat = nj_matrix(n, mu)
    nj = pyramid(n, mu);                % friction cone normal vectors
    nj_mat = zeros(n*4,3*4);            % 4 contact points
    for i = 1:4
        ind_j = i + 2*(i-1);
        ind_i = n*(i-1) + 1;
        % dot(fi,nij) = |fi| |nij| cos(theta) >= 0
            % where theta is the angle between fi and nij
            % if    theta <= 90 deg     then cos(theta) >= 0
            % if    theta  > 90 deg     then cos(theta)  < 0
        nj_mat(ind_i:ind_i+n-1, ind_j:ind_j+2) = nj';
    end
end
