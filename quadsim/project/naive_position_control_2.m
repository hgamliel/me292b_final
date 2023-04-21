function [out,use_torque,kp,kd] = naive_position_control_2(t)
    % THIS CONTROLLER DOES NOT WORK
    % directly command limb joint angles through gait cycle

    % target phase angles = [hip, thigh, calf]
    traj_FR = [-0.1,  0.8,  -1.5;
               -0.1,  0.8,    -2;
               -0.1, 0.45, -1.75;
               -0.1,  0.6,  -1.5;
               -0.1,  0.8,  -1.5];
    traj_RL = [0.1,  0.8,  -1.5;
               0.1,  0.8,    -2;
               0.1, 0.45, -1.75;
               0.1,  0.6,  -1.5;
               0.1,  0.8,  -1.5];
    traj_FL = [0.1, 0.45, -1.75;
               0.1,  0.6,  -1.5;
               0.1,  0.8,  -1.5;
               0.1,  0.8,  -1.5;
               0.1,  0.8,    -2];
    traj_RR = [-0.1, 0.45, -1.75;
               -0.1,  0.6,  -1.5;
               -0.1,  0.8,  -1.5;
               -0.1,  0.8,  -1.5;
               -0.1,  0.8,    -2];

    tf = 0.5;               % time to take one step [s]
    t_steps = linspace(0, tf, size(traj_FR,1))';
    t = mod(t, tf);

    % goal joint positions
    FR_pos = interp1(t_steps, traj_FR, t, 'pchip');
    RL_pos = interp1(t_steps, traj_RL, t, 'pchip');
    FL_pos = interp1(t_steps, traj_FL, t, 'pchip');
    RR_pos = interp1(t_steps, traj_RR, t, 'pchip');

    out = [FR_pos FL_pos RR_pos RL_pos];
    use_torque = false;
    kp = 100*ones(1,12);
    kd = 1*ones(1,12);
end
