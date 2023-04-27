function simulate_brachiation(u_array, T_release)
    if nargin == 0
        n = 500;                % points in time
        u_array = zeros(n,2);   % input torques
        T_release = 1;          % time of release [s]
    end
    tau_max = 100;              % actuator torque limit [N*m]

    % stance initial conditions: q = [th1; th2]
    q0 = [-2.3072 + deg2rad(90); -1.6271];
    dq0 = [0; 0];
    s0 = [q0; dq0];
    
    % simulate stance dynamics before release
    options = odeset('Events', @(t, s) release_event(t, s, T_release));
    Tf = 3; Tspan = [0 Tf];
    param.n = n, param.u_array = u_array; param.Tf = Tf;
    [stance_t, stance_s] = ...
        ode45(@(t, s) stance_dynamics(t, s, param), Tspan, s0, options);

    % simulate flight dynamics after release
    % hand/pivot at release: x = y = dx = dy = 0
    s0 = [[0; 0; stance_s(end,1:2)']; [0; 0; stance_s(end,3:4)']];
    Tspan = [stance_t(end) Tf];
    [flight_t, flight_s] = ...
        ode45(@(t, s) flight_dynamics(t, s, param), Tspan, s0);
    
    animate_brachiation(stance_s, flight_s);
    % movie_brachiation(stance_s, flight_s);
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

function animate_brachiation(stance_s, flight_s)
    figure

    for i = 1:length(stance_s)  % stance positions
        s = stance_s(i,:)';
        p1 = stance_p1_gen(s);
        p2 = stance_p2_gen(s);

        pts = [0 0;
               p1';
               p2'];

        plot(pts(:,1), pts(:,2), '-o', 'linewidth', 1.5)
        axis([-3 3 -3 3])
        grid on
        pause(0.05)
    end

    for i = 1:length(flight_s)  % flight positions
        s = flight_s(i,:)';
        p0 = flight_p0_gen(s);
        p1 = flight_p1_gen(s);
        p2 = flight_p2_gen(s);

        pts = [p0';
               p1';
               p2'];

        plot(pts(:,1), pts(:,2), '-o', 'linewidth', 1.5)
        axis([-3 3 -3 3])
        grid on
        pause(0.05)
    end

    close all
end

function M = movie_brachiation(stance_s, flight_s)
    frames = length(stance_s) + length(flight_s);
    M(frames) = struct('cdata', [], 'colormap', []);

    h = figure;
    h.Visible = 'off';

    for i = 1:length(stance_s)  % stance positions
        s = stance_s(i,:)';
        p1 = stance_p1_gen(s);
        p2 = stance_p2_gen(s);

        pts = [0 0;
               p1';
               p2'];

        plot(pts(:,1), pts(:,2), '-o', 'linewidth', 1.5)
        axis([-3 3 -3 3])
        grid on
        M(i) = getframe(h);
    end

    for i = 1:length(flight_s)  % flight positions
        s = flight_s(i,:)';
        p0 = flight_p0_gen(s);
        p1 = flight_p1_gen(s);
        p2 = flight_p2_gen(s);

        pts = [p0';
               p1';
               p2'];

        plot(pts(:,1), pts(:,2), '-o', 'linewidth', 1.5)
        axis([-3 3 -3 3])
        grid on
        j = length(stance_s) + i;
        M(j) = getframe(h);
    end

    % play animation
    h.Visible = 'on';
    movie(h, M, 5);
end
