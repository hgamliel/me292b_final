function objcost = obj_brachiation(stance_s, flight_s, param)
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
    th10 = stance_s(1,1);
    th1f = flight_s(end,3);
    rot_ang = abs(th1f - th10);

    %% goal 3: periodic gait constraint (optional)
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

    th_diff = th1_diff + th2_diff;
    dth = dth1f + dth2f;

    %% cost function
    % energy cost J
    % u_norm2 = vecnorm(uData,2,2).^2;
    % xT_I = sData(end,1);
    % objcost = (1/xT_I)*trapz(tData,u_norm2);
    % objcost = dist + -rot_ang; % add weights

    % var_fncount = var_fncount + 1;
    % % display function count and save iteration details
    % if mod(var_fncount,400)==0
    %     display(var_fncount);
    %     save;
    % end

end