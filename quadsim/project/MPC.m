function [out,use_torque,kp,kd] = MPC(t,v,omega,quat,q,dq,foot_contact)
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
    % joint limitations (deg): hip: -46~46, thigh: -60~240, calf: -154.5~-52.5

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

    %% model parameters
    INIT_POS = [0, 0, 0.32];
    INIT_QUAT = euler_to_quaternion([0 0 0]);

    g = 9.81;                   % acceleration due to gravity [m/s^2]
    m = 108/g;                  % body mass [kg]
    Ib = [0.24,    0, 0;
             0, 0.80, 0;
             0,    0, 1];       % body inertia in body frame [kg*m^2]
    I = I_hat(Ib, quat);        % body inertia in world frame [kg*m^2]

    global HIP_OFFSETS
    COM_OFFSET = -[0.012731, 0.002186, 0.000515];
    HIP_OFFSETS = [ 0.183, -0.047, 0;
                    0.183,  0.047, 0;
                   -0.183, -0.047, 0;
                   -0.183,  0.047, 0] + COM_OFFSET;

    init_joint_angles = [-.1, .8, -1.5, ...
                          .1, .8, -1.5, ...
                         -.1, .8, -1.5, ...
                          .1, .8, -1.5];

    %% initialize and update variables
    global pos
    v_des = 0.1;                % desired forward velocity [m/s]
    dx = update_position(t, v, v_des);

    % gait: 3 legs in stance, 1 leg in swing
    global swing_time_start gait_phase swing_traj swing_t_steps
    if (t == 0)
        swing_time_start = 0;
        gait_phase = 0;         % footfall order: 0:RL, 1:FL, 2:RR, 3:FR

        swing_traj = [0.1,  0.8,  -1.5;
                      0.1,  0.8,    -2;
                      0.1, 0.45, -1.75;
                      0.1,  0.6,  -1.5;
                      0.1,  0.8,  -1.5];
        tf = 0.5;               % time to take one step [s]
        swing_t_steps = linspace(0, tf, size(swing_traj,1))';
    end

    % leg_id: Leg0 FR, Leg1 FL, Leg2 RR, Leg3 RL
    switch gait_phase
        case 0      % RL
            sw_ft_id = 3;
        case 1      % FL
            sw_ft_id = 1;
        case 2      % RR
            sw_ft_id = 2;
        case 3      % FR
            sw_ft_id = 0;
    end

    % if minimum swing time has passed and swing foot is back on ground
    swing_time_elapsed = t - swing_time_start;
    if (swing_time_elapsed > 0.05) && (foot_contact(sw_ft_id+1) == 1)
        gait_phase = mod(gait_phase+1, 4);
        swing_time_start = t;
    end

    % joint angles during swing phase
    swing_joint_pos = ...
        interp1(swing_t_steps, swing_traj, swing_time_elapsed, 'pchip');
    if mod(sw_ft_id, 2) == 0
        % reverse hip abduction for right leg
        swing_joint_pos(1) = -swing_joint_pos;
    end

    %% dynamics
    R = quaternion_to_rotm(quat);
    foot_pos = foot_positions_in_base_frame(q);
    p0FR = foot_pos(1,:); p1FL = foot_pos(2,:);
    p2RR = foot_pos(3,:); p3RL = foot_pos(4,:);

    A = [zeros(3), zeros(3),       R', zeros(3);
         zeros(3), zeros(3), zeros(3),   eye(3);
         zeros(3), zeros(3), zeros(3), zeros(3);
         zeros(3), zeros(3), zeros(3), zeros(3)];
    B = [        zeros(3),         zeros(3),         zeros(3),         zeros(3);
                 zeros(3),         zeros(3),         zeros(3),         zeros(3);
         inv(I)*hat(p0FR), inv(I)*hat(p1FL), inv(I)*hat(p2RR), inv(I)*hat(p3RL);
                 eye(3)/m,         eye(3)/m,         eye(3)/m,         eye(3)/m];

    % state-space form with additional gravity state
    % x = [theta; p; omega; pdot] --> x = [theta; p; omega; pdot; g]
    Ac = zeros(13); Bc = zeros(13,12);
    Ac(1:12,1:12) = A; Bc(1:12,1:12) = B; Ac(12,13) = -1;

    % state function
    QuadrupedStateFcn = @(x, u) Ac*x + Bc*u;
    QuadrupedStateJacobianFcn = @(x0, u0) deal(numerical_Jacobian(Ac, Bc, x0, u0));

    %% state vector and desired state
    % state
    omega = R*omega;
    x = [euler; pos'; omega'; v'; g];

    COM_des = [pos(1)+dx; 0; 0.32];
    % mostly interested in moving forward along X, other errors can be ignored
    % can weight only linear velocity instead of position
    x_des = [zeros(3,1); COM_des; zeros(3,1)'; [v_des; 0; 0]];
    nlobj.Weights.OutputVariables = ...
        [[0.1, 0.1, 0.1] [0, 0, 0] [0.1, 0.1, 0.1] [1, 0.5, 0.5]];

    %% MPC control, run through dynamics to get contact forces
    nx = 13;                    % number of states
    ny = 13;                    % number of outputs
    nu = 12;                    % number of inputs
    nlobj = nlmpc(nx, ny, nu);  % nonlinear MPC object

    % nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
    % test MPC static balancing
    % linear MpC documentation


    % specify state function and Jacobian state function
    nlobj.Model.StateFcn = @QuadrupedStateFcn;
    nlobj.Jacobian.StateFcn = @QuadrupedStateJacobianFcn;

    % specify sample time [s], prediction horizon [steps], control horizon [2 steps]
    nlobj.Ts = 0.1;
    nlobj.PredictionHorizon = 18;
    nlobj.ControlHorizon = 2;

    % eliminate contact force from swing foot, restrict gravity state
    min_lim = [repmat({-inf},12,1); g];
    max_lim = [repmat({inf},12,1); g];

    ind = sw_ft_id + 1; ind = ind + 2*(ind-1);
    min_lim(ind:ind+2) = {0}; max_lim(ind:ind+2) = {0};

    nlobj.MV = struct('Min', min_lim, 'Max', max_lim);

    % ground contact constraints
    nlobj.Optimization.CustomIneqConFcn = @groundContactIneqConFcn;

    global lastMV
    if (t == 0)
        lastMV = zeros(12,1);
    end
    [uk,~,~] = nlmpcmove(nlobj,x,lastMV,x_des,[],nloptions);
    lastMV = uk;
    fc = uk;                    % optimal contact forces

    % for feet not in contact with ground, need swing phase code
        % convert joint angles/pos to torques - how?
        % FIND HOW RUN.PY COMPUTES TORQUES FOR UK AND GENERAL JOINT POS

    % tau = Jacobian transpose * contact force
    J = cell(4,1);
    for leg_id = 0:3
        J{leg_id+1} = ComputeJacobian(leg_id, q);
    end

    tau = zeros(12,1);
    for i = 1:4
        ind = i + 2*(i-1);
        tau(ind:ind+2) = -J{i}' * fc(ind:ind+2);
    end

    % output torques in correct order????
    sw_tau = zeros(3,1);
    switch sw_ft_id
        case 0      % FR
            % out = [sw_tau' tau'];
            tau(1:3) = sw_tau;
        case 1      % FL
            % out = [tau(1:3)' sw_tau' tau(4:9)'];
            tau(4:6) = sw_tau;
        case 2      % RR
            % out = [tau(1:6)' sw_tau' tau(7:9)'];
            tau(7:9) = sw_tau;
        case 3      % RL
            % out = [tau' sw_tau'];
            tau(10:12) = sw_tau;
    end
    use_torque = true;          % enable torque control
end

%% controller helper functions

% computes rotation matrix from quaternion
function rotm = quaternion_to_rotm(quat)
    q0 = quat(4); q1 = quat(1); q2 = quat(2); q3 = quat(3);
    
    rotm = [2*(q0*q0+q1*q1)-1,   2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2);
              2*(q1*q2+q0*q3), 2*(q0*q0+q2*q2)-1,   2*(q2*q3-q0*q1);
              2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1), 2*(q0*q0+q3*q3)-1];
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
    quat = [qx; qy; qz; qw];        % quat = w + xi + yj + zk
end

% computes Euler angles from quaternion
function euler = quaternion_to_euler(quat)
    x = quat(1); y = quat(2); y = quat(3); z = quat(4);

    t0 = 2*(w*x + y*z);
    t1 = 1 - 2*(x*x + y*y);
    roll_x = atan2(t0, t1);
 
    t2 = 2*(w*y - z*x);
    t2 = min(1, max(-1, t2));
    pitch_y = asin(t2);
 
    t3 = 2*(w*z + x*y);
    t4 = 1 - 2*(y*y + z*z);
    yaw_z = atan2(t3, t4);
 
    euler = [yaw_z; pitch_y; roll_x];
end

% approximate inertia in world frame from body inertia in body frame
function I = I_hat(I_body, quat)
    R = quaternion_to_rotm(quat);
    I = R* I_body * R';

    % small roll and pitch angles approximation
    % euler = quaternion_to_euler(quat); yaw = euler(3);
    % Ryaw  = @(y) [cos(y) sin(y) 0; -sin(y) cos(y) 0; 0 0 1]';
    % I = Ryaw(yaw) * I_body * Ryaw(yaw)';
end

% computes hat map of vector
function x_hat = hat(x)
    % x_hat = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    x_hat = skew(x);
end

% calculate numerical Jacobian of state function
function A_jac, B_jac = numerical_Jacobian(A, B, x0, u0)
    % Jacobian functor
    J = @(x, h, F) (F(repmat(x,size(x'))+diag(h))-F(repmat(x,size(x'))))./h';

    % state functions
    f1 = @(x) A*x;      % + constant B*u (for constant u)
    f2 = @(x) B*u;      % + constant A*x (for constant x)

    % points at which to estimate Jacobian
    % x0, u0

    % Step to take on each dimension (has to be small enough for precision)
    h = 1e-5*ones(size(x));

    % Compute the jacobian
    A_jac = J(x0, h, f1);
    B_jac = J(u0, h, f2);
end

% inequality function for MPC, ground contact constraints
function cineq = groundContactIneqConFcn(X,U,e,data)
    mu = 0.8;                           % friction coefficient
    n = 10;                             % friction cone pyramid, # sides           
    nj_mat = nj_matrix(n, mu);          % friction cone normal vectors

    A = zeros(4,12);

    % unilateral constraint fi_z >= 0
    A(sub2ind(size(A),1:4,3:3:12)) = -1;

    % friction cone constraint
    A = [A; -nj_mat];

    cineq = A*X;
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

%% robot helper functions

function joint_angles = foot_position_in_hip_frame_to_joint_angle(foot_position, l_hip_sign)
    l_up = 0.2;                     % thigh length [m]
    l_low = 0.2;                    % calf length [m]
    l_hip = 0.08505*l_hip_sign;     % hip length [m]
    x = foot_position(1); y = foot_position(2); z = foot_position(3);

    theta_knee = -acos( (x^2 + y^2 + z^2 - l_hip^2 - l_low^2 - l_up^2) / ...
        (2*l_low*l_up) );
    l = sqrt( l_up^2 + l_low^2 + 2*l_up*l_low*cos(theta_knee) );
    theta_hip = asin(-x/l) - theta_knee/2;
    c1 = l_hip*y - l*cos(theta_hip + theta_knee/2)*z;
    s1 = l*cos(theta_hip + theta_knee/2)*y + l_hip*z;
    theta_ab = atan2(s1, c1);

    joint_angles = [theta_ab, theta_hip, theta_knee];
end

function foot_position = foot_position_in_hip_frame(angles, l_hip_sign)
    l_up = 0.2;                     % thigh length [m]
    l_low = 0.2;                    % calf length [m]
    l_hip = 0.08505*l_hip_sign;     % hip length [m]
    theta_ab = angles(1); theta_hip = angles(2); theta_knee = angles(3);
    leg_distance = sqrt(l_up^2 + l_low^2 + 2*l_up*l_low*cos(theta_knee));
    eff_swing = theta_hip + theta_knee/2;
    
    off_x_hip = -leg_distance*sin(eff_swing);
    off_z_hip = -leg_distance*cos(eff_swing);
    off_y_hip = l_hip;

    off_x = off_x_hip;
    off_y = cos(theta_ab)*off_y_hip - sin(theta_ab)*off_z_hip;
    off_z = sin(theta_ab)*off_y_hip + cos(theta_ab)*off_z_hip;

    foot_position = [off_x, off_y, off_z];
end

% computes analytical Jacobian
function J = analytical_leg_jacobian(leg_angles, leg_id)
    % leg_angles: a list of 3 numbers for current abduction, hip and knee angle
    % leg_id: Leg0 FR, Leg1 FL, Leg2 RR, Leg3 RL
    % l_hip_sign: whether it's a left (1) or right (-1) leg.

    l_hip_sign = (-1)^(leg_id + 1);
    l_up = 0.2;                     % thigh length [m]
    l_low = 0.2;                    % calf length [m]
    l_hip = 0.08505*l_hip_sign;     % hip length [m]

    t1 = leg_angles(1); t2 = leg_angles(2); t3 = leg_angles(3);
    l_eff = sqrt(l_up^2 + l_low^2 + 2*l_up*l_low*cos(t3));
    t_eff = t2 + t3/2;
    
    J = zeros(3);
    J(1,1) = 0;
    J(1,2) = -l_eff*cos(t_eff);
    J(1,3) = l_low*l_up*sin(t3)*sin(t_eff)/l_eff - l_eff*cos(t_eff)/2;
    J(2,1) = -l_hip*sin(t1) + l_eff*cos(t1)*cos(t_eff);
    J(2,2) = -l_eff*sin(t1)*sin(t_eff);
    J(2,3) = -l_low*l_up*sin(t1)*sin(t3)*cos(t_eff)/l_eff - l_eff*sin(t1)*sin(t_eff)/2;
    J(3,1) = l_hip*cos(t1) + l_eff*sin(t1)*cos(t_eff);
    J(3,2) = l_eff*sin(t_eff)*cos(t1);
    J(3,3) = l_low*l_up*sin(t3)*cos(t1)*cos(t_eff)/l_eff + l_eff*sin(t_eff)*cos(t1)/2;
end

function foot_positions = foot_positions_in_base_frame(foot_angles)
  foot_angles = reshape(foot_angles,3,4)';
  foot_positions = zeros(4,3);

  for i = 1:4
      l_hip_sign = (-1)^i;
      foot_positions(i,:) = ...
          foot_position_in_hip_frame(foot_angles(i,:), l_hip_sign);
  end

  global HIP_OFFSETS
  foot_positions = foot_positions + HIP_OFFSETS;
  fpB = fpH + hip_offsets
end

% computes Jacobian for a given leg
% leg_id: Leg0 FR, Leg1 FL, Leg2 RR, Leg3 RL
function J = ComputeJacobian(leg_id, q)
    idx = 3*leg_id + 1;
    motor_angles = q(idx:idx+2);
    J = analytical_leg_jacobian(motor_angles, leg_id);
end
