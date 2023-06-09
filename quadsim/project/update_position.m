function dx = update_position(t, v, v_des)
    % velocity integrator to get position (x forward, y left, z up)
    global pos time prev_time INIT_POS
    
    if (t == 0) % && ~isempty(pos)
        pos = INIT_POS;     % initial position
        prev_time = 0;
    end

    time = t;
    dt = time - prev_time;
    pos = pos + v*dt;
    prev_time = t;

    % return desired x translation
    dx = v_des*dt;
end