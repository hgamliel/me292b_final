function [out,use_torque,kp,kd] = naive_position_control(t, v)
    % THIS CONTROLLER DOES NOT WORK
    % directly command limb joint angles through gait cycle
    % initial joint angles (rad) = [0 0.9 -1.8];
    init_joint_angles = [-.1, .8, -1.5, ...
                          .1, .8, -1.5, ...
                         -.1, .8, -1.5, ...
                          .1, .8, -1.5];

    % target phase angles = [hip, thigh, calf]
        up = deg2rad([0 42 32]);
        front = deg2rad([0 26 15]);
        down = deg2rad([0 35 8]);
        back = deg2rad([0 42 -1]);
    % up = [0.1, 0.8+0.1, -1.5-0.35];
    % front = [0.1, 0.8-0.15, -1.5-0.15];
    % down = [0.1, 0.8, -1.5];
    % back = [0.1, 0.8+0.1, -1.5+0.15];

    % time interval for each phase
    DelT = 0.1;
    state = mod(floor(t/DelT),4);

    % display estimated position over time
    global pos
    v_des = 0.1;                % desired forward velocity [m/s]
    update_position(t, v, v_des);

    % set joint angles
    out = zeros(1,12);
    switch state
        case 0
            % RR leg up, FR leg down, RL down, FL leg up
            out(7:9) = up; out(1:3) = down; out(10:12) = down; out(4:6) = up;
            out(1) = -out(1); out(7) = -out(7);
        case 1
            % RR leg front, FR leg back, RL back, FL leg front
            out(7:9) = front; out(1:3) = back; out(10:12) = back; out(4:6) = front;
            out(1) = -out(1); out(7) = -out(7);
        case 2
            % RR leg down, FR leg up, RL up, FL leg down
            out(7:9) = down; out(1:3) = up; out(10:12) = up; out(4:6) = down;
            out(1) = -out(1); out(7) = -out(7);
        case 3
            % RR leg back, FR leg front, RL front, FL leg back
            out(7:9) = back; out(1:3) = front; out(10:12) = front; out(4:6) = back;
            out(1) = -out(1); out(7) = -out(7);
    end

    % out = [-.1, .8, -2.35, ...
    %     .1, .8, -2.35, ...
    %    -.1, .8, -2.35, ...
    %     .1, .8, -2.35];
    use_torque = false;
    kp = 100*ones(1,12);
    kd = 0*ones(1,12);

    % attempt to make the robot jump
    % out = [-.1, .8, 0, ...
    %         .1, .8, 0, ...
    %        -.1, .8, 0, ...
    %         .1, .8, 0];
end
