function simulate_brachiation()
    tau_mas = 100;              % actuator torque limit [N*m]

    % stance initial conditions: q = [th1; th2]
    q0 = [-2.3072; -1.6271];
    dq0 = [0; 0];
    s0 = [q0; dq0];

    % input torques
    u = [0; 0];
    
    % simulate stance dynamics
    options = odeset('Events', @release_event);
    Tf = 10; Tspan = [0 Tf];
    [st_t_sol, st_s_sol] = ...
        ode45(@(t, s) stance_dynamics(t, s, u), Tspan, s0, options);

    % simulate flight dynamics after release
    % hand/pivot at release: x = y = dx = dy = 0
    s0 = [0; 0; st_s_sol(end,1:2)'; 0; 0; st_s_sol(end,3:4)'];
    Tspan = [st_t_sol(end) Tf];
    [fl_t_sol, fl_s_sol] = ...
        ode45(@(t, s) flight_dynamics(t, s, u), Tspan, s0, options);
    
    animate_brachiation(st_s_sol, fl_s_sol);
    % movie_brachiation(st_s_sol, fl_s_sol);
end

function ds = stance_dynamics(t, s, u)
    ds = stance_dyn_gen(s, u);
end

function ds = flight_dynamics(t, s, u)
    ds = flight_dyn_gen(s, u);
end

function [value, isterminal, direction] = release_event(t, s)
    T_release = 1;              % time of release [s]

    value = t - T_release;      % detect when event occurs (value == 0)
    isterminal = 1;             % stop when event occurs
    direction = 1;              % zero crossing from negative to postive
end

function animate_brachiation(st_s_sol, fl_s_sol)
    figure

    for i = 1:length(st_s_sol)  % stance positions
        s = st_s_sol(i,:)';
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

    for i = 1:length(fl_s_sol)  % flight positions
        s = fl_s_sol(i,:)';
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
end

function M = movie_brachiation(st_s_sol, fl_s_sol)
    frames = length(st_s_sol) + length(fl_s_sol);
    M(frames) = struct('cdata', [], 'colormap', []);

    h = figure;
    h.Visible = 'off';

    for i = 1:length(st_s_sol)  % stance positions
        s = st_s_sol(i,:)';
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

    for i = 1:length(fl_s_sol)  % flight positions
        s = fl_s_sol(i,:)';
        p0 = flight_p0_gen(s);
        p1 = flight_p1_gen(s);
        p2 = flight_p2_gen(s);

        pts = [p0';
               p1';
               p2'];

        plot(pts(:,1), pts(:,2), '-o', 'linewidth', 1.5)
        axis([-3 3 -3 3])
        grid on
        j = length(st_s_sol) + i;
        M(j) = getframe(h);
    end

    % play animation
    h.Visible = 'on';
    movie(h, M, 5);
end
