function simulate_brachiation(u_array, T_release)
    if nargin == 0
        n = 500;                % points in time
        u_array = zeros(n,2);   % input torques
        T_release = 1;          % time of release [s]
    else
        n = length(u_array);
    end
    targetPos = [3 0];          % position of target branch
    % RL paper: exponential distance reward, terminate early
    param.n = n; param.u_array = u_array; param.targetPos = targetPos;

    tau_max = 500;              % actuator torque limit [N*m]

    % stance initial conditions: q = [th1; th2]
    q0 = [-2.3072 + deg2rad(90); -1.6271];
    dq0 = [0; 0];
    s0 = [q0; dq0];
    
    % simulate stance dynamics before release
    options = odeset('Events', @(t, s) release_event(t, s, T_release));
    Tf = 3; Tspan = [0 Tf]; param.Tf = Tf;
    [stance_t, stance_s] = ...
        ode45(@(t, s) stance_dynamics(t, s, param), Tspan, s0, options);

    % simulate flight dynamics after release
    % hand/pivot at release: x = y = dx = dy = 0
    s0 = [[0; 0; stance_s(end,1:2)']; [0; 0; stance_s(end,3:4)']];
    Tspan = [stance_t(end) Tf];
    [flight_t, flight_s] = ...
        ode45(@(t, s) flight_dynamics(t, s, param), Tspan, s0);
    
    axis_limits = [-2 4 -3 3]; param.axis_limits = axis_limits;
    animate_brachiation(stance_t, stance_s, flight_t, flight_s, param);
    % movie_brachiation(stance_t, stance_s, flight_t, flight_s, param);
end

function ds = stance_dynamics(t, s, param)
    u = interpolate_input(t, param);
    ds = stance_dyn_gen(s, u);
end

function ds = flight_dynamics(t, s, param)
    u = interpolate_input(t, param);
    ds = flight_dyn_gen(s, u);
end

% computes input torques at given time from input array
function u = interpolate_input(t, param)
    n = param.n; Tf = param.Tf; u_array = param.u_array;
    times = linspace(0, Tf, n);
    u = interp1(times, u_array, t)';
end

function [value, isterminal, direction] = release_event(t, s, T_release)
    value = t - T_release;      % detect when event occurs (value == 0)
    isterminal = 1;             % stop when event occurs
    direction = 1;              % zero crossing from negative to postive
end

function animate_brachiation(stance_t, stance_s, flight_t, flight_s, param)
    Tf = param.Tf; n = round(Tf/0.025); targetPos = param.targetPos;
    axis_limits = param.axis_limits;

    p0 = []; p1 = []; p2 = [];
    for i = 1:length(stance_s)-1    % stance positions
        s = stance_s(i,:)';
        p0(end+1,:) = [0 0];
        p1(end+1,:) = stance_p1_gen(s)';
        p2(end+1,:) = stance_p2_gen(s)';
    end
    for i = 1:length(flight_s)      % flight positions
        s = flight_s(i,:)';
        p0(end+1,:) = flight_p0_gen(s)';
        p1(end+1,:) = flight_p1_gen(s)';
        p2(end+1,:) = flight_p2_gen(s)';
    end

    figure
    data_times = [stance_t(1:end-1); flight_t];
    times = linspace(0, Tf, n);
    for i = 1:n                 % interpolate to ensure same size time steps
        t = times(i);
        p012 = interp1(data_times, [p0 p1 p2], t);
        pts = [p012(1:2);
               p012(3:4);
               p012(5:6)];

        hold on
        plot(pts(1:2,1), pts(1:2,2), '-o', 'linewidth', 1.5)
        plot(pts(2:3,1), pts(2:3,2), '-o', 'linewidth', 1.5)
        plot(targetPos(1), targetPos(2), 'ks', 'linewidth', 1.5)
        axis(axis_limits)
        grid on
        % drawnow;
        pause(0.025)
        clf
    end

    close all
end

function M = movie_brachiation(stance_t, stance_s, flight_t, flight_s, param)
    Tf = param.Tf; n = round(Tf/0.025); targetPos = param.targetPos;
    axis_limits = param.axis_limits;

    p0 = []; p1 = []; p2 = [];
    for i = 1:length(stance_s)-1    % stance positions
        s = stance_s(i,:)';
        p0(end+1,:) = [0 0];
        p1(end+1,:) = stance_p1_gen(s)';
        p2(end+1,:) = stance_p2_gen(s)';
    end
    for i = 1:length(flight_s)      % flight positions
        s = flight_s(i,:)';
        p0(end+1,:) = flight_p0_gen(s)';
        p1(end+1,:) = flight_p1_gen(s)';
        p2(end+1,:) = flight_p2_gen(s)';
    end

    data_times = [stance_t(1:end-1); flight_t];
    times = linspace(0, Tf, n);

    frames = n;
    M(frames) = struct('cdata', [], 'colormap', []);

    h = figure;
    h.Visible = 'off';

    for i = 1:n                 % interpolate to ensure same size time steps
        t = times(i);
        p012 = interp1(data_times, [p0 p1 p2], t);
        pts = [p012(1:2);
               p012(3:4);
               p012(5:6)];

        hold on
        plot(pts(1:2,1), pts(1:2,2), '-o', 'linewidth', 1.5)
        plot(pts(2:3,1), pts(2:3,2), '-o', 'linewidth', 1.5)
        plot(targetPos(1), targetPos(2), 'ks', 'linewidth', 1.5)
        axis(axis_limits)
        grid on
        M(i) = getframe(h);
        clf
    end

    % play animation
    h.Visible = 'on';
    movie(h, M, 5);
end
