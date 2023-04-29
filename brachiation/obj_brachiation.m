function objcost = obj_brachiation(u_array, stance_s, flight_s, flight_t, param)
    %% goal 1: reach target
    targetPos = param.targetPos;    % position of target branch

    % positions of swing hand
    n = length(stance_s) + length(flight_s);
    pos2 = zeros(n,2);

    for i = 1:length(stance_s)      % stance positions
        s = stance_s(i,:)';
        p2 = stance_p2_gen(s);
        pos2(i,:) = p2';
    end

    for i = 1:length(flight_s)      % flight positions
        s = flight_s(i,:)';
        p2 = flight_p2_gen(s);
        j = length(stance_s) + i;
        pos2(i,:) = p2';
    end

    % closest distance to target
    dist = min(vecnorm(pos2-targetPos, 2, 2));

    %% goal 2: do a backflip
    th20 = flight_s(1,4);           % th2 at release
    th2f = flight_s(end,4);
    rot_ang = abs(th2f - th20);

    %% goal 3: minimize energy cost
    n = param.n; Tf = param.Tf;
    times = linspace(0, flight_t(end), n);
    u_norm2 = vecnorm(u_array,2,2).^2;
    u_norm2_vals = interp1(linspace(0, Tf, n), u_norm2, times);
    u_int = trapz(times, u_norm2_vals);
    
    %% cost function
    a1 = 10; a2 = 1; a3 = 0.1;          % cost function weights
    objcost = a1*dist + a2*-rot_ang + a3*u_int;

end