function objcost = obj_brachiation(stance_s, flight_s)
    %% goal 1: reach target
    targetPos = [1 0];          % position of target branch

    % positions of swing hand
    n = length(stance_s) + length(flight_s);
    pos2 = zeros(n,2);

    for i = 1:length(stance_s)  % stance positions
        s = stance_s(i,:)';
        p2 = stance_p2_gen(s);
        pos2(i,:) = p2';
    end

    for i = 1:length(flight_s)  % flight positions
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

    %% constraints (add in separate function?)
    % start and end in mirrored configuration? not needed if we're not
    % looking for periodic gait
    % make T_release another parameter to optimize for, separate from the
    % input u

end