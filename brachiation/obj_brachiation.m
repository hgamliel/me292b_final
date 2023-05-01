function objcost = obj_brachiation(stance_s, flight_s, flight_t, param)
    %% goal 1: reach target
    targetPos = param.targetPos;    % position of target branch

    % position of either hand
    n = length(stance_s) + length(flight_s);
    pos2 = zeros(n,2);

    for i = 1:length(stance_s)      % stance positions
        s = stance_s(i,:)';
        p2 = stance_p2_gen(s);
        pos2(i,:) = p2';
    end

    for i = 1:length(flight_s)      % flight positions
        s = flight_s(i,:)';
        p0 = s(1:2);
        p2 = flight_p2_gen(s);
        j = length(stance_s) + i;
        pos0(j,:) = p0';
        pos2(j,:) = p2';
    end

    % closest distance to target
    dist0 = min(vecnorm(pos0-targetPos, 2, 2));
    dist2 = min(vecnorm(pos2-targetPos, 2, 2));
    dist = min(dist0, dist2);

    %% goal 2: do a backflip
    th10 = flight_s(1,3);           % th1 at release
    th1f = flight_s(end,3);
    rot_ang = abs(th1f - th10)/(2*pi);
    flip = abs(rot_ang - 1);        % under/overshoot from one flip

    %% goal 3: minimize energy cost
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

    % integral torque-squared
    u_norm2 = u1.^2 + u2.^2;
    u_int = trapz(times, u_norm2);
    
    %% cost function
    a1 = 100; a2 = 0.01; a3 = 1e-5/2;   % cost function weights
    objcost = a1*dist + a2*flip + a3*u_int;
end

% computes input torques from sum of sines
function u = torques_from_sum_of_sines(t, param)
    n = param.n; u_coeff = param.u_coeff;

    % divide coefficient array in half
    N = length(u_coeff/2); coeff1 = u_coeff(1:N); coeff2 = u_coeff(N+1:end);

    % extract coefficients
    coeff1 = reshape(coeff1, [], 3); coeff2 = reshape(coeff2, [], 3);
    a1 = coeff1(:,1); b1 = coeff1(:,2); c1 = coeff1(:,3);
    a2 = coeff2(:,1); b2 = coeff2(:,2); c2 = coeff2(:,3);
    
    % compute input torques
    u1 = sum(a1 .* sin(b1.*t + c1));
    u2 = sum(a2 .* sin(b2.*t + c2));

    % bound the torques
    tau_max = 500;                  % actuator torque limit [N*m]
    u1 = min(max(u1, -tau_max), tau_max);
    u2 = min(max(u2, -tau_max), tau_max);

    u = [u1; u2];
end
