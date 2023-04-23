function [out, use_torque, kp, kd] = joint_pos_from_RL_control(t)
    [q_goal, dq_goal, t_steps] = RL_quadruped_params();

    delay = 0.25;       % delay to allow robot to move into initial pos [s]

    if t < delay        % move robot into initial position
        out = q_goal(1,:);
    else                % follow RL joint trajectories
        time = t - delay;
        out = interp1(t_steps, q_goal, time, 'linear');
    end

    use_torque = false;
    kp = 100*ones(1,12);
    kd = 1*ones(1,12);
end

% loads joint trajectories from MATLAB's RL quadruped example
function [q_goal, dq_goal, t_steps] = RL_quadruped_params()
    % load results from RL quadruped example
    load('RL_quad_sim_results.mat')
    obs = experience.Observation.observations.Data;
    q = obs([13 15, 19 21, 25 27, 31 33],:);
    dq = obs([13 15, 19 21, 25 27, 31 33]+1,:);
    
    % original order:
    % FL [thigh calf], FR [thigh calf], RL [thigh calf], RR [thigh calf]
    % CCW positive

    % data normalized between -1 and 1
        % hip and knee joint angle limits
        d2r = pi/180;
        q_hip_min = -120 * d2r; q_hip_max = -30 * d2r;
        q_knee_min = 60 * d2r; q_knee_max = 140 * d2r;
        % hip and knee joint angular speed limit
        w_max = 2*pi*60/60; w_min = -2*pi*60/60;

    % unnormalize data: xnorm = 2*( (xi – xmin) / (xmax – xmin) ) – 1
    q(1:2:end,:) = (q(1:2:end,:)+1) / 2 * (q_hip_max-q_hip_min) + q_hip_min;
    q(2:2:end,:) = (q(2:2:end,:)+1) / 2 * (q_knee_max-q_knee_min) + q_knee_min;
    dq = (dq+1) / 2 * (w_max-w_min) + w_min;
    
    % reorder for A1 quadruped simulation
    n = length(obs);
    q_goal = [[zeros(n,1) -q(1,:)' -q(2,:)'], [zeros(n,1) -q(3,:)' -q(4,:)'], ...
        [zeros(n,1) -q(5,:)' -q(6,:)'], [zeros(n,1) -q(7,:)' -q(8,:)']];
    dq_goal = [[zeros(n,1) -dq(1,:)' -dq(2,:)'], [zeros(n,1) -dq(3,:)' -dq(4,:)'], ...
        [zeros(n,1) -dq(5,:)' -dq(6,:)'], [zeros(n,1) -dq(7,:)' -dq(8,:)']];

    Ts = 0.025;     % sample time [s]
    Tf = 10;        % simulation time [s]
    Tf = Tf*0.48;   % faster gait

    t_steps = linspace(0, Tf, n)';
end
