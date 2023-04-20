function [out,use_torque,kp,kd] = contact_force_control(t,v,omega,quat,q,dq,foot_contact)
    %% Inputs
    %%% t %%%
    % Time elapsed in the simulation [s]
    % Float type

    %%% v %%%
    % Robot base linear velocity [m/s]
    % [x, y, z], dim=3

    %%% omega %%%
    % Robot base angular velocity [rad/s]
    % [x, y, z], dim=3

    %%% quat %%%
    % Robot base orientation in quaternion [-]
    % [x, y, z, w], dim=4

    %%% q %%%
    % Joint position [rad]
    % [front_right_hip, front_right_thigh, front_right_calf,
    %   front_left_hip,  front_left_thigh,  front_left_calf,
    %   rear_right_hip,  rear_right_thigh,  rear_right_calf,
    %    rear_left_hip,   rear_left_thigh,   rear_left_calf]
    % dim=12

    %%% dq %%%
    % Joint velocity [rad/s]
    % Order and dimension same as q

    %%% foot_contact %%%
    % Robot foot contact [-] (1 stands for contact, 0 non-contact)
    % [front_right, front_left, rear_right, rear_left], dim=4

    %% Outputs
    %%% out %%%
    % Output (joint torque [Nm] or position [rad])
    % Order and dimension same as q

    %%% use_torque %%%
    % If use torque output (otherwise use position)

    %%% kp %%%
    % Desired joint kp values (valid only if use position control)
    % Order and dimension same as q

    %%% kd %%%
    % Desired joint kd values (valid only if use position control)
    % Order and dimension same as q

    %%
    global pos
    v_des = 0.1;            % desired forward velocity [m/s]
    dx = update_position(t, v, v_des);

    COM_des = [pos(1) + dx; pos(2); pos(3)];
    % contact sequence?

    % contact force control, PD law and quadprog to get contact forces

    % for feet not in contact with ground, need swing phase code

    % Jacobian transpose

    %% generalized coordinates and velocities
    q = s(1:model.n);
    dq = s(model.n+1:2*model.n);

    % model parameters
    m = model.M;            % robot mass [kg]
    mu = 0.8;               % friction coefficient
    g = [0; 0; 9.81];       % acceleration due to gravity [m/s^2]

    % COM positions and body orientations
    drc_d = zeros(3,1);
    R_d = eye(3);
    Omega_d = zeros(3,1);
    rc_d(2) = -rc_d(2)*5;

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
    kh = 900*4.5; kv = 3000*4.5; Kp = diag([kh kh*2 kv]);
    dh = sqrt(m*kh)*2*0.8; dv = sqrt(m*kv)*2*0.2; Kd = diag([dh dh*2 dv]);
    f_d = -Kp*(rc - rc_d) - Kd*(drc - drc_d) + m*g;     % + m*ddrc_d;

    Kr = 2000*3; Dr = 1000*8;           % rotational stiffness and damping
    Rdb = R_d'*Rb; Rwb = Rb;
    quat = rotm_to_quaternion(Rdb); delta = quat(1); epsilon = quat(2:4);
    tau_r = -2*(delta*eye(3) + hat(epsilon))*Kr*epsilon;
    % error definition 1: quaternion
    tau_d = Rwb*(tau_r - Dr*(Omega - Omega_d));
    % error definition 2: rotation matrix
    % tau_d = -Kr*error(Rb,R_d) - Dr*(Omega - Rb'*R_d*Omega_d); % + I*dOmega_d
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

% computes rotation matrix from quaternion
function rotm = quaternion_to_rotm(quat)
    q0 = quat(4); q1 = quat(1); q2 = quat(2); q3 = quat(3);
    
    rotm = [2*(q0*q0+q1*q1)-1,   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
              2*(q1*q2+q0*q3), 2*(q0*q0+q2*q2)-1,   2*(q2*q3-q0*q1);
              2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1), 2*(q0*q0+q3*q3)-1];
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
