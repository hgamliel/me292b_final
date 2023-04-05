function tau = contact_force_control(s, model)
    q = s(1:20);
    dq = s(21:40);

    % model parameters
    m = model.M;            % robot mass [kg]
    g = [0; 0; 9.81];       % acceleration due to gravity [m/s^2]

    % COM positions and body orientations
    rc_d = [-0.0103; 0; 0.8894];
    drc_d = zeros(3,1);
    R_d = eye(3);
    Omega_d = zeros(3,1);

    [rc, drc] = computeComPosVel(q, dq, model);
    yaw = q(4); pitch = q(5); roll = q(6);
    R = euler_to_rotm(yaw,pitch,roll);
    R = R';
    Omega = [dq(4); dq(5); dq(6)];

    %% 1) stabilizing wrench at COM
    Kp = 500; Kd = 100;     % proportional and differential gain
    f_d = -Kp*(rc - rc_d) - Kd*(drc - drc_d) + m*g;     % + m*ddrc_d;

    Kr = 100; Dr = 50;      % rotational stiffness and damping
    quat = euler_to_quaternion(yaw,pitch,roll);
    delta = quat(1); epsilon = quat(2:4); Rdb = R_d'*R;
    tau_r = -2*(delta*eye(3) + hat(epsilon))*Kr*epsilon;
    tau_d = Rdb*(tau_r - Dr*(Omega - Omega_d));

    F_d = [f_d; tau_d];

    %% 2) contact forces to produce desired wrench
    Gc = contact_grasp_map(s, model, rc);

    n = 4;                              % friction cone pyramid, # sides           
    nj_mat = nj_matrix(n);              % friction cone normal vectors

    A = zeros(4,12); A(sub2ind(size(A),1:4,3:3:12)) = -1;
    b = zeros(4,1);                     % unilateral constraint fi_z >= 0
    A = [A; -nj_mat];
    b = [b; zeros(n*4,1)];              % friction cone constraint
    Aeq = []; beq = [];
    % instead of using equality constraint (Aeq*x = beq; Gc*fc = F_d),
    % add ||F_d - Gc*fc||^2 to the cost function with weights prioritizing:
    % minimizing ||fc||^2 << getting correct tau_d << getting correct f_d
    a1 = 10; a2 = 1; a3 = 0.1;          % a3 << a2 << a1
    A1 = [eye(3) zeros(3)];             % extract forces:  [I 0] ||F_d - Gc*fc||^2
    A2 = [zeros(3) eye(3)];             % extract torques: [0 I] ||F_d - Gc*fc||^2
    Gc1 = A1*Gc; F_d1 = A1*F_d; Gc2 = A2*Gc; F_d2 = A2*F_d; 
    H = Gc1'*Gc1*2*a1 + Gc2'*Gc2*2*a2 + a3*2*eye(12);
    f = (-2*a1*F_d1'*Gc1)' + (-2*a2*F_d2'*Gc2)' + a3*zeros(12,1);
    lb = []; ub = []; x0 = [];
    options =  optimset('TolCon',1e3,'Display','off');
    % options =  optimset('TolCon',1e3);
    options =  optimset('Display','off');
    fc = quadprog(H,f,A,b,Aeq,beq,...
        lb,ub,x0,options);              % min 0.5*x'*H*x + f'*x
    % fc = Gc\F_d;                        % TRY THIS BY ITSELF FOR DEBUGGING

    %% 3) motor torques to produce desired contact forces
    [J1f, J1b, J2f, J2b] = compute_foot_Jacobians(s, model);
    J = {J1f J1b J2f J2b};
    % A = zeros(20,1); A([7:14 19:20]) = 1; A = diag(A);
    % A = [A(7:14,:); A(19:20,:)];        % to select actuated joint angles
    A = [zeros(10,6) eye(10)];          % to select actuated joint angles

    tau = zeros(10,1);
    for i = 1:4
        ind = i + 2*(i-1);
        % coordinate transformation: T*q + d
        % inv(T')*D*inv(T); inv(T')*C*inv(T); inv(T')*G; inv(T')*B;
        % but T = I if only change is translating q(1:3) from hip to COM
        % T not identity, COM is a function of other q, redefine q to calc COM
        Jbar = J{i};
        Jtilde = A*Jbar';
        tau = tau + -Jtilde * [fc(ind:ind+2)];
    end
end

% computes body orientation from Euler angles
function R = euler_to_rotm(yaw, pitch, roll)
    Rroll  = @(r) [1 0 0; 0 cos(r) sin(r); 0 -sin(r) cos(r)];
    Rpitch = @(p) [cos(p) 0 -sin(p); 0 1 0; sin(p) 0 cos(p)];
    Ryaw   = @(y) [cos(y) sin(y) 0; -sin(y) cos(y) 0; 0 0 1];
    R = Rroll(roll)*Rpitch(pitch)*Ryaw(yaw);
end

% computes quaternion from Euler angles
function quat = euler_to_quaternion(yaw, pitch, roll)
    y = yaw; p = pitch; r = roll;   % rotation sequence: yaw-pitch-roll

    qx = sin(r/2)*cos(p/2)*cos(y/2) - cos(r/2)*sin(p/2)*sin(y/2);
    qy = cos(r/2)*sin(p/2)*cos(y/2) + sin(r/2)*cos(p/2)*sin(y/2);
    qz = cos(r/2)*cos(p/2)*sin(y/2) - sin(r/2)*sin(p/2)*cos(y/2);
    qw = cos(r/2)*cos(p/2)*cos(y/2) + sin(r/2)*sin(p/2)*sin(y/2);

    % qw = scalar component, [qx; qy; qz] = vector component
    quat = [qw; qx; qy; qz];        % quat = w + xi + yj + zk
end

% [unused] computes vector error between rotation matrices
% https://stackoverflow.com/questions/6522108/error-between-two-rotations
% function err = error(R, Rdes)
%     err = rotmat2vec3d(R*Rdes')';
% end

% computes contact grasp map
function Gc = contact_grasp_map(s, model, rc)
    [p1, p2, p3, p4] = computeFootPositions(s, model);
    p = [p1 p2 p3 p4] - rc; % positions of contact points relative to COM

    Gc = zeros(6,3*4);
    Rpo = eye(3);
    for i = 1:4
        ind = i + 2*(i-1);
        Gc(1:3,ind:ind+2) = Rpo;
        Gc(4:6,ind:ind+2) = hat(p(:,i))*Rpo;
    end
end

% computes hat map of vector
function x_hat = hat(x)
    % x_hat = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    x_hat = skew(x);
end

% computes normal vectors for friction cone pyramid approximation
function nj = pyramid(n)
    mu = 0.8;           % coefficient of friction
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
function nj_mat = nj_matrix(n)
    nj = pyramid(n);                    % friction cone normal vectors
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

% compute 3 x (20 - 4) foot Jacobians (last 10 = actuated joints)
function [J1f, J1b, J2f, J2b] = compute_foot_Jacobians(s, model)
    q = s(1:20);
    dq = s(21:40);
    
    [~, ~, G] = model.gamma_q(model, q, dq) ;
     
    [J1f, ~] = bodyJac_vel(model, model.idx.foot1, q, xlt(model.p1)) ;
    J1f = J1f*G;
    
    [J1b, ~] = bodyJac_vel(model, model.idx.foot1, q, xlt(model.p2)) ;
    J1b = J1b*G;
    
    [J2f, ~] = bodyJac_vel(model, model.idx.foot2, q, xlt(model.p1)) ;
    J2f = J2f*G;
    
    [J2b, ~] = bodyJac_vel(model, model.idx.foot2, q, xlt(model.p2)) ;
    J2b = J2b*G;
end
