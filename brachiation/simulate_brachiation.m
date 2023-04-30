function objcost = simulate_brachiation(x, param_animate)
    if nargin == 0
        % no input arguments, run default values
        T_release = 0.5;        % time of release [s]
        u_coeff = zeros(1,6);   % input torques coefficients
        u_coeff = [1500 0 pi/2 0 0 pi/2];
        % u_coeff = [1000 0 pi/2 10 0 pi/2]; T_release = 0.6;
        param_animate = true;
        max_flight_step = 1e-3; % integrate ode45 at finer time step 
    elseif nargin == 1
        % x = [T_release, u_coeff]
        T_release = x(1);
        u_coeff = x(2:end);
        param_animate = false;
        max_flight_step = 1;
    else
        % x = [T_release, u_coeff], input 2nd parameter to animate
        T_release = x(1);
        u_coeff = x(2:end);
        param_animate = true;
        max_flight_step = 1e-3;
    end
    targetPos = [3 0];          % position of target branch
    param.u_coeff = u_coeff; param.targetPos = targetPos;
    param.tau_max = 1500;       % actuator torque limit [N*m]
    param.max_flight_step = max_flight_step;

    % stance initial conditions: q = [th1; th2]
    q0 = [-2.3072 + deg2rad(90); -2.3072 + deg2rad(90) - 1.6271];
    % q0 = [-pi/2; -pi/2];
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
    param.tol = 0.075;          % tolerance for reaching target
    options = odeset('Events', @(t, s) flight_event(t, s, param), ...
        'MaxStep', max_flight_step);
    Tspan = [stance_t(end) Tf];
    [flight_t, flight_s] = ...
        ode45(@(t, s) flight_dynamics(t, s, param), Tspan, s0, options);
    
    % animation
    if param_animate == true
        % simulate dynamics after landing/contact, just for visualization
        [land_t, land_pos] = add_landing(flight_t, flight_s, param);

        axis_limits = [-2 4 -3 3]; param.axis_limits = axis_limits;
        animate_brachiation(stance_t, stance_s, flight_t, flight_s, land_t, land_pos, param);
        % movie_brachiation(stance_t, stance_s, flight_t, flight_s, land_t, land_pos, param);

        % plot input torques
        plot_torques(flight_t, param);
    end

    % cost function
    objcost = obj_brachiation(stance_s, flight_s, flight_t, param);
end

function ds = stance_dynamics(t, s, param)
    u = torques_from_sum_of_sines(t, param);
    ds = stance_dyn_gen(s, u);
end

function ds = flight_dynamics(t, s, param)
    u = torques_from_sum_of_sines(t, param);
    ds = flight_dyn_gen(s, u);
end

% computes input torques from sum of sines
function u = torques_from_sum_of_sines(t, param)
    u_coeff = param.u_coeff;

    % divide coefficient array in half
    u_coeff = reshape(u_coeff, [], 2);
    coeff1 = u_coeff(:,1); coeff2 = u_coeff(:,2);

    % extract coefficients
    coeff1 = reshape(coeff1, [], 3); coeff2 = reshape(coeff2, [], 3);
    a1 = coeff1(:,1); b1 = coeff1(:,2); c1 = coeff1(:,3);
    a2 = coeff2(:,1); b2 = coeff2(:,2); c2 = coeff2(:,3);
    
    % compute input torques
    u1 = sum(a1 .* sin(b1.*t + c1));
    u2 = sum(a2 .* sin(b2.*t + c2));

    % bound the torques
    tau_max = param.tau_max;
    u1 = min(max(u1, -tau_max), tau_max);
    u2 = min(max(u2, -tau_max), tau_max);

    u = [u1; u2];
end

function [value, isterminal, direction] = release_event(t, s, T_release)
    value = t - T_release;      % detect when event occurs (value == 0)
    isterminal = 1;             % stop when event occurs
    direction = 1;              % zero crossing from negative to postive
end

function [value, isterminal, direction] = flight_event(t, s, param)
    % detect if either hand has reached target branch
    targetPos = param.targetPos; tol = param.tol;
    p0 = s(1:2)'; p2 = flight_p2_gen(s)';
    dist0 = max(norm(p0-targetPos), tol); dist2 = max(norm(p2-targetPos), tol);

    % detect multiple events: end simulation if robot falls out of bounds
    p2x = p2(1); p2y = p2(2); LB = -6; UB = 6;
    p2x_check = min(max(p2x, LB), UB); p2y_check = min(max(p2y, LB), UB);

    value = [dist0 - tol; dist2 - tol; abs(p2x_check) - UB; abs(p2y_check) - UB];
    isterminal = [1; 1; 1; 1];  % stop when event occurs
    direction = [0; 0; 0; 0];   % direction of zero crossing
end

% returns land positions to end of flight state, if robot lands
function [land_t, land_pos] = add_landing(flight_t, flight_s, param)
    s = flight_s(end,:)';

    % ending positions of hands
    targetPos = param.targetPos; tol = param.tol;
    p0 = s(1:2)'; p2 = flight_p2_gen(s)';
    dist0 = norm(p0 - targetPos); dist2 = norm(p2 - targetPos);

    % determines if any hand has made contact
    if (dist0 <= tol) || (dist2 <= tol)
        % pass
    else
        % neither hand has made contact
        land_t = [];
        land_pos = [];
        return;
    end

    % determine which hand has made contact
    % computes initial conditions after landing, to input into stance dynamics
    if dist0 < dist2            % p0 contact
        s0 = [s(3:4); s(7:8)];
    else                        % p2 contact
        p1 = flight_p1_gen(s);
        th1 = atan2(p1(2)-p2(2), p1(1)-p2(1)) + deg2rad(90);
        th2 = atan2(p0(2)-p1(2), p0(1)-p1(1)) + deg2rad(90);
        s0 = [th1; th2; s(8); s(7)];
    end

    % simulate dynamics after landing/contact
    options = odeset('MaxStep', param.max_flight_step);
    Tspan = [flight_t(end) flight_t(end)+0.2];
        [land_t, land_s] = ode45(@(t, s) land_dynamics(t, s, param), Tspan, s0);

    % land positions
    p0 = []; p1 = []; p2 = [];
    for i = 1:length(land_s)
        s = land_s(i,:)';
        p0(end+1,:) = [0 0] + targetPos;
        p1(end+1,:) = stance_p1_gen(s)' + targetPos;
        p2(end+1,:) = stance_p2_gen(s)' + targetPos;
    end

    if dist0 < dist2            % p0 contact
        land_pos = [p0 p1 p2];
    else                        % p2 contact
        land_pos = [p2 p1 p0];
    end
end

function ds = land_dynamics(t, s, param)
    u = [0; 0];
    ds = stance_dyn_gen(s, u);
end

% simulation animation
function animate_brachiation(stance_t, stance_s, flight_t, flight_s, land_t, land_pos, param)
    temp = [stance_t; flight_t; land_t]; Tf = temp(end);
    n = round(Tf/0.025); targetPos = param.targetPos;
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
    if ~isempty(land_pos)           % land positions
        p0 = [p0; land_pos(2:end,1:2)];
        p1 = [p1; land_pos(2:end,3:4)];
        p2 = [p2; land_pos(2:end,5:6)];
    end

    figure
    data_times = [stance_t(1:end-1); flight_t; land_t(2:end)];
    times = linspace(0, Tf, n);
    for i = 1:n                 % interpolate to ensure same size time steps
        clf
        t = times(i);
        p012 = interp1(data_times, [p0 p1 p2], t);
        pts = [p012(1:2);
               p012(3:4);
               p012(5:6)];
        prev_pos = interp1(data_times, [p0 p1 p2], times(max(1,i-10):i));

        hold on
        plot(pts(1:2,1), pts(1:2,2), '-o', 'linewidth', 1.5)
        plot(pts(2:3,1), pts(2:3,2), '-o', 'linewidth', 1.5)
        plot(pts(2,1), pts(2,2), 'o', 'linewidth', 1.5, 'MarkerFaceColor', '#EDB120')
        plot(prev_pos(:,1), prev_pos(:,2), 'color', '#0072BD')
        plot(prev_pos(:,5), prev_pos(:,6), 'color', '#D95319')
        plot(targetPos(1), targetPos(2), 'ks', 'linewidth', 1.5)
        axis(axis_limits)
        grid on
        % drawnow;
        pause(0.025)
    end
end

% create movie frames of simulation animation, save to video
function M = movie_brachiation(stance_t, stance_s, flight_t, flight_s, land_t, land_pos, param)
    temp = [stance_t; flight_t; land_t]; Tf = temp(end);
    n = round(Tf/0.025); targetPos = param.targetPos;
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
    if ~isempty(land_pos)           % land positions
        p0 = [p0; land_pos(2:end,1:2)];
        p1 = [p1; land_pos(2:end,3:4)];
        p2 = [p2; land_pos(2:end,5:6)];
    end

    data_times = [stance_t(1:end-1); flight_t; land_t(2:end)];
    times = linspace(0, Tf, n);

    frames = n;
    M(frames) = struct('cdata', [], 'colormap', []);

    h = figure;
    h.Visible = 'off';

    % save movie frames to video file
    filename = regexprep(char(datetime), '[^a-zA-Z0-9]', '_');
    v = VideoWriter(filename);
    v.FrameRate = 15;
    open(v)

    for i = 1:n                 % interpolate to ensure same size time steps
        clf
        t = times(i);
        p012 = interp1(data_times, [p0 p1 p2], t);
        pts = [p012(1:2);
               p012(3:4);
               p012(5:6)];
        prev_pos = interp1(data_times, [p0 p1 p2], times(max(1,i-20):i));

        hold on
        plot(pts(1:2,1), pts(1:2,2), '-o', 'linewidth', 1.5)
        plot(pts(2:3,1), pts(2:3,2), '-o', 'linewidth', 1.5)
        plot(pts(2,1), pts(2,2), 'o', 'linewidth', 1.5, 'MarkerFaceColor', '#EDB120')
        plot(prev_pos(:,1), prev_pos(:,2), 'color', '#0072BD')
        plot(prev_pos(:,5), prev_pos(:,6), 'color', '#D95319')
        plot(targetPos(1), targetPos(2), 'ks', 'linewidth', 1.5)
        axis(axis_limits)
        grid on
        M(i) = getframe(h);
        writeVideo(v, M(i));
    end

    % play animation
    h.Visible = 'on';
    movie(h, M, 1);

    close(v)
end

% plots torques from coefficients
function plot_torques(flight_t, param)
    u_coeff = param.u_coeff;

    % divide coefficient array in half
    u_coeff = reshape(u_coeff, [], 2);
    coeff1 = u_coeff(:,1); coeff2 = u_coeff(:,2);

    % extract coefficients
    coeff1 = reshape(coeff1, [], 3); coeff2 = reshape(coeff2, [], 3);
    a1 = coeff1(:,1); b1 = coeff1(:,2); c1 = coeff1(:,3);
    a2 = coeff2(:,1); b2 = coeff2(:,2); c2 = coeff2(:,3);

    % replicate coefficients for vector-wise function
    n = 100;
    times = linspace(0, flight_t(end), n)';
    a1 = repmat(a1', n, 1); b1 = repmat(b1', n, 1); c1 = repmat(c1', n, 1);
    a2 = repmat(a2', n, 1); b2 = repmat(b2', n, 1); c2 = repmat(c2', n, 1);

    % compute torques and apply saturation limit
    tau_max = param.tau_max;
    u1 = sum(a1 .* sin(b1.*times + c1), 2);
    u2 = sum(a2 .* sin(b2.*times + c2), 2);
    u1(u1 > tau_max) = tau_max; u1(u1 < -tau_max) = -tau_max;
    u2(u2 > tau_max) = tau_max; u2(u2 < -tau_max) = -tau_max;

    % plot
    for figureN = 1:1000
        if ishandle(figureN)
            % pass if figure number already exists
        else
            break;
        end
    end
    figure(figureN)
    hold on
    plot(times, u1, 'linewidth', 1.5)
    plot(times, u2, 'linewidth', 1.5)
    xlabel('time (s)')
    ylabel('torque (N\cdotm)')
    title('u(t)')
    legend('u_1','u_2','location','best')
    grid on
end
