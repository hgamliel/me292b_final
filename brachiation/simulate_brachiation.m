function objcost = simulate_brachiation(x, param_animate)
    % no input arguments
    if nargin == 0
        n = 500;                % points in time
        T_release = 1;          % time of release [s]
        u_array = zeros(n,2);   % input torques
        param_animate = true;
    % T_release and u_array specified as vector x
    elseif nargin == 1
        T_release = x(1);
        n = round(length(x(2:end))/2);
        u_array = reshape(x(2:end), n, 2);
        param_animate = false;
    else
    % T_release and u_array specified as vector x, input 2nd parameter to animate
        T_release = x(1);
        n = round(length(x(2:end))/2);
        u_array = reshape(x(2:end), n, 2);
        param_animate = true;
    end
    targetPos = [3 0];          % position of target branch
    % RL paper: exponential distance reward, terminate early
    param.n = n; param.u_array = u_array; param.targetPos = targetPos;

    % stance initial conditions: q = [th1; th2]
    q0 = [-2.3072 + deg2rad(90); -2.3072 + deg2rad(90) - 1.6271];
    dq0 = [0; 0];
    s0 = [q0; dq0];
    
    % simulate stance dynamics before release
    options = odeset('Events', @(t, s) release_event(t, s, T_release));
    Tf = 10; Tspan = [0 Tf]; param.Tf = Tf;
    [stance_t, stance_s] = ...
        ode45(@(t, s) stance_dynamics(t, s, param), Tspan, s0, options);

    % simulate flight dynamics after release
    % hand/pivot at release: x = y = dx = dy = 0
    s0 = [[0; 0; stance_s(end,1:2)']; [0; 0; stance_s(end,3:4)']];
    options = odeset('Events', @(t, s) flight_event(t, s, targetPos));
    Tspan = [stance_t(end) Tf];
    [flight_t, flight_s] = ...
        ode45(@(t, s) flight_dynamics(t, s, param), Tspan, s0, options);
    
    % animation
    if param_animate == true
        axis_limits = [-2 4 -3 3]; param.axis_limits = axis_limits;
        animate_brachiation(stance_t, stance_s, flight_t, flight_s, param);
        % movie_brachiation(stance_t, stance_s, flight_t, flight_s, param);
    end

    % cost function
    objcost = obj_brachiation(u_array, stance_s, flight_s, flight_t, param);
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

function [value, isterminal, direction] = flight_event(t, s, targetPos)
    % detect if hand has reached target branch
    p2 = flight_p2_gen(s)'; dist = norm(p2 - targetPos); tol = 0.01;

    % detect multiple events: end simulation if robot falls out of bounds
    p2x = p2(1); p2y = p2(2); LB = -4; UB = 4;
    p2x_check = min(max(p2x, LB), UB); p2y_check = min(max(p2y, LB), UB);

    value = [dist - tol; abs(p2x_check) - 4; abs(p2y_check) - 4];
    isterminal = [1; 1; 1];     % stop when event occurs
    direction = [-1; 0; 0];     % direction of zero crossing
end

function animate_brachiation(stance_t, stance_s, flight_t, flight_s, param)
    Tf = flight_t(end); n = round(Tf/0.025); targetPos = param.targetPos;
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
    Tf = flight_t(end); n = round(Tf/0.025); targetPos = param.targetPos;
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
