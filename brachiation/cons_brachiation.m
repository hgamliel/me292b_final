function [cineq, ceq] = cons_brachiation(x, param_animate)
    if nargin == 0
        n = 500;                % points in time
        T_release = 1;          % time of release [s]
        u_array = zeros(n,2);   % input torques
        param_animate = true;
    else
        T_release = x(1);
        n = round(length(x(2:end))/2);
        u_array = reshape(x(2:end), n, 2);
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
    Tf = 5; Tspan = [0 Tf]; param.Tf = Tf;
    [stance_t, stance_s] = ...
        ode45(@(t, s) stance_dynamics(t, s, param), Tspan, s0, options);

    % simulate flight dynamics after release
    % hand/pivot at release: x = y = dx = dy = 0
    s0 = [[0; 0; stance_s(end,1:2)']; [0; 0; stance_s(end,3:4)']];
    options = odeset('Events', @(t, s) flight_event(t, s, targetPos));
    Tspan = [stance_t(end) Tf];
    [flight_t, flight_s] = ...
        ode45(@(t, s) flight_dynamics(t, s, param), Tspan, s0, options);

    %% periodic gait constraint (optional)
    % This goal might be too hard to optimize for. For simplicity, we could
    % say that finding a periodic gait could be one of the ways to extend
    % the project for future research.

    % start and end in mirrored configuration
    th10 = stance_s(1,1); th1f = flight_s(end,3);
    th20 = stance_s(1,2); th2f = flight_s(end,4);

    th1_goal = deg2rad(180) - abs(th20);
    th2_goal = deg2rad(180) - abs(th10);

    th1_diff = mod(abs(th1f-th1_goal), 2*pi);
    th2_diff = mod(abs(th2f-th2_goal), 2*pi);

    % (start and) end with zero angular velocity
    dth1f = flight_s(end,7);
    dth2f = flight_s(end,8);

    ceq = [th1_diff;
           th2_diff;
              dth1f;
              dth2f];
    cineq = [];
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
