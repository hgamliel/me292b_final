function tau = virtual_constraints_control(s, model)
    %% generalized coordinates and velocities
    q = s(1:model.n);
    dq = s(model.n+1:2*model.n);

    % coordinate variable names
    % q1 = hip roll/abduction, q2 = hip yaw/rotation, q3 = hip pitch/flexion
    % q4 = knee pitch/joint, q5 = shin pitch/knee spring
    % q6 = tarsus pitch/ankle joint, q7 = toe pitch/joint
    qx = q(1); qy = q(2); qz = q(3); qyaw = q(4); qpitch = q(5); qroll = q(6);
    q1L = q(7); q1R = q(8); q2L = q(9); q2R = q(10); q3L = q(11); q3R = q(12);
    q4L = q(13); q4R = q(14); q5L = q(15); q5R = q(16);
    q6L = q(17); q6R = q(18); q7L = q(19); q7R = q(20);

    dqx = dq(1); dqy = dq(2); dqz = dq(3); dqyaw = dq(4); dqpitch = dq(5); dqroll = dq(6);
    dq1L = dq(7); dq1R = dq(8); dq2L = dq(9); dq2R = dq(10); dq3L = dq(11); dq3R = dq(12);
    dq4L = dq(13); dq4R = dq(14); dq5L = dq(15); dq5R = dq(16);
    dq6L = dq(17); dq6R = dq(18); dq7L = dq(19); dq7R = dq(20);

    %% virtual constraints
    % variables being regulated
    h0_ = [qroll;
           q2R;
           qpitch;
           q4R;
           q1L;
           q2L;
           q3L;
           q4L;
           q7L];
    dh0_ = [dqroll;
            dq2R;
            dqpitch;
            dq4R;
            dq1L;
            dq2L;
            dq3L;
            dq4L;
            dq7L];

    % initial state = desired state
    q0 = getInitialState(model);
    qroll0 = q0(6); q2R0 = q0(10); qpitch0 = q0(5); q4R0 = q0(14);
    q1L0 = q0(7); q2L0 = q0(9); q3L0 = q0(11); q4L0 = q0(13); q7L0 = q0(19);
    q7R0 = q0(20);

    % desired constraints
    hd1 = qroll0;                                   % qroll, torso roll
    hd2 = q2R0;                                     % q2st, stance hip yaw
    hd3 = qpitch0;                                  % qpitch, torso pitch
    hd4 = sqrt(0.5292*cos(q4R0 + 0.035) + 0.5301);  % qLLst, stance leg length
    hd5 = qroll0 + q1L0;                            % qLRsw, swing leg roll
    hd6 = q2L0;                                     % q2sw, swing hip yaw
    hd7 = -qpitch0 + q3L0 - ...
        acos( 0.5*(cos(q4L0+0.035)+0.5292) / sqrt(0.5292*cos(q4L0+0.035)+0.5301)) ...
        + 0.1;                                      % qLPsw, swing leg pitch
    hd8 = sqrt(0.5292*cos(q4L0 + 0.035) + 0.5301);  % qLLsw, swing leg length
    hd9 = -qpitch0 + q7L0 + 1.1;                    % qFPsw, swing foot pitch

    hd_ = [hd1;
           hd2;
           hd3;
           -acos(1.8896*(hd4^2)-1.0017) - 0.035;
           hd5 - qroll0;
           hd6;
           hd7 + qpitch0 - 0.1 + acos( 0.5*(cos(q4L0+0.035)+0.5292) / sqrt(0.5292*cos(q4L0+0.035)+0.5301) );
           -acos(1.8896*(hd8^2)-1.0017) - 0.035;
           hd9 + qpitch0 - 1.1];
    hd_ = [qroll0;
           q2R0;
           qpitch0;
           q4R0;
           q1L0;
           q2L0;
           q3L0;
           q4L0;
           q7L0];
    dhd_ = zeros(9,1);

    % adding last stance leg actuator to virtual constraints
    hd10 = -qpitch0 + q7R0 + 1.1;           % qFPst, stance foot pitch
    h0_ = [h0_;
           q7R];
    dh0_ = [dh0_;
            dq7R];
    % hd_ = [hd_;
    %        hd10 + qpitch0 - 1.1];
    hd_ = [hd_;
           q7R0];
    dhd_ = [dhd_;
            0];

    % virtual constraints
    y_ = h0_ - hd_;
    dy_ = dh0_ - dhd_;

    %% control
    % parameter tuning
    Kp_abduction = 400; Kp_rotation = 200; Kp_thigh = 200;
    Kp_knee = 500; Kp_toe = 600;
    Kd_abduction = 4; Kd_rotation = 4; Kd_thigh = 10;
    Kd_knee = 20; Kd_toe = 80;

    % proportional and differential gain matrices
    Kp = diag([Kp_abduction, Kp_rotation, Kp_thigh, Kp_knee, ...
               Kp_abduction, Kp_rotation, Kp_thigh, Kp_knee, Kp_toe, ...
               Kp_toe]);
    Kd = diag([Kd_abduction, Kd_rotation, Kd_thigh, Kd_knee, ...
               Kd_abduction, Kd_rotation, Kd_thigh, Kd_knee, Kd_toe, ...
               Kd_toe]);

    % actuated leg joints: q1, q2, q3, q4, q7
    % u = [q1R; q2R; q3R; q4R; q1L; q2L; q3L; q4L; q7L; q7R]
    u = -Kp*y_ - Kd*dy_;
    uq1R = u(1); uq2R = u(2); uq3R = u(3); uq4R = u(4);
    uq1L = u(5); uq2L = u(6); uq3L = u(7); uq4L = u(8); uq7L = u(9);
    uq7R = u(10);

    % tau = [q1L; q1R; q2L; q2R; q3L; q3R; q4L; q4R; q7L; q7R]
    tau = [uq1L; uq1R; uq2L; uq2R; uq3L; uq3R; uq4L; uq4R; uq7L; uq7R];
end
