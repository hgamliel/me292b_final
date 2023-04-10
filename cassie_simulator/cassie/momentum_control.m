function tau = momentum_control(s, model)
    % generalized coordinates and velocities
    q = s(1:model.n);
    dq = s(model.n+1:2*model.n);

    % model parameters
    m = model.M;            % robot mass [kg]
    I = I_COM(s, model);    % robot moment of inertia [kg*m^2]
    mu = 0.8;               % friction coefficient
    g = [0; 0; -9.81];      % acceleration due to gravity [m/s^2]

    % COM positions, angular velocity
    rG_d = [-0.0103; 0; 0.8894];
    drG_d = zeros(3,1);

    [rG, drG] = computeComPosVel(q, dq, model);
    Jdq = bodyJac(model, model.idx.torso, q)*dq;
    Omega = Jdq(1:3);

    %% desired momenta for balance maintenance
    % linear momentum
    Gamma11 = diag([40 20 40]); Gamma12 = diag([8 3 8]);
    dl_d = Gamma11*(drG_d - drG) + Gamma12*(rG_d - rG);
    dl_d = m*dl_d;

    % centroidal angular momentum
    k = I*Omega; k_d = zeros(3,1);
    Gamma21 = diag([20 20 20]);
    dk_d = Gamma21*(k_d - k);

    %% foot ground reaction force and center of pressure
    % dl = mg + fl + fr
    % dk = dk_f + dk_m
        % dk_f = (rl - rG) x fl + (rr - rG) x fr
        % dk_m = tau_l + tau_r = tau(9) + tau(10)

    % a) determine foot GRFs: fc = [fl; fr]
    n = 10;                         % friction cone pyramid, # sides           
    nj_mat = nj_matrix(n, mu);      % friction cone normal vectors

    A = zeros(2,6); A(sub2ind(size(A),1:2,3:3:6)) = -1;
    b = zeros(2,1);                 % unilateral constraint fi_z >= 0
    A = [A; -nj_mat];
    b = [b; zeros(n*2,1)];          % friction cone constraint

    [rl, rr] = compute_ankle_locations(q, model);
    L = [eye(3) eye(3)];            % L*fc = f1f + f1b + f2f + f2b
                                    % minimize ||(dl_d - mg) - L*fc||^2
    K = [hat(rl-rG) hat(rr-rG)];    % K*fc = dk_f
                                    % minimize ||dk_d - K*fc||^2

    % cost function with weights prioritizing:
    % linear momentum >> angular momentum >>  minimizing ||fc||^2 
    w = 0.1; epsilon = 0.01;        % cost function weights
    H = L'*L*2 + K'*K*2*w + epsilon*2*eye(6);
    f = (-2*(dl_d - m*g)'*L)' + (-2*w*dk_d'*K)' + epsilon*zeros(6,1);
    Aeq = []; beq = []; lb = []; ub = []; x0 = [];
    options =  optimset('Display','off');
    fc = quadprog(H,f,A,b,Aeq,beq,...
        lb,ub,x0,options);          % min 0.5*x'*H*x + f'*x

    % b) determine foot CoPs: eta = [dlx dly dlz drx dry drz]'
        % ignore ground reaction torque (flat ground)
    [rlf, rlb, rrf, rrb] = computeFootPositions(s, model);
    p = [rlf-rl rlb-rl rrf-rr rrb-rr];
    dlx_ub = p(1,1); dlx_lb = p(1,2);   % line contact: dly = 0, fixed
    drx_ub = p(1,3); drx_lb = p(1,4);   % line contact: dry = 0, fixed
    dlz_ub = p(3,2); dlz_lb = p(3,1);   % ankle height from ground
    drz_ub = p(3,4); drz_lb = p(3,3);   % ankle height from ground
    hl = mean([dlz_ub dlz_lb]); hr = mean([drz_ub drz_lb]);

    % dk_md = dk_d - dk_f = tau_l + tau_r, where tau_i = -fi x di
    epsilon = 0.1;
    fl = fc(1:3); fr = fc(4:6);
    dk_f = K*fc; dk_md = dk_d - dk_f;
    Psi = [-hat(fl) -hat(fr)];
    eta_d = [(hl/fl(3))*fl; (hr/fr(3))*fr];
    H = Psi'*Psi*2 + 2*epsilon*eye(6);
    f = (-2*dk_md'*Psi)' + -2*epsilon*eta_d;
    
    A = []; b = []; Aeq = []; beq = []; x0 = [];
    lb = [dlx_lb 0 dlz_lb drx_lb 0 drz_lb];
    ub = [dlx_ub 0 dlz_ub drx_ub 0 drz_ub];
    options =  optimset('Display','off');
    eta = quadprog(H,f,A,b,Aeq,beq,...
        lb,ub,x0,options);          % min 0.5*x'*H*x + f'*x

    % admissible momenta rate changes
    dl_a = m*g + fl + fr;
    dk_m = Psi*eta;
    dk_a = dk_m + dk_f;

    %% joint accelerations and torques
    dh_a = [dk_a; dl_a];
    tau = 0;
end

% calculates moment of inertia about Cassie COM
function I = I_COM(s, model)
    I_COM = zeros(3);
    for i = 1:model.NB
        [mass, com_offset, I] = mcI_inv(model, i);
        R = com_offset;
        if (mass > 0)
            I_b = I + mass*(norm(R)^2*eye(3) - R*R');
            I_COM = I_COM + I_b;
        end
    end
end

% computes ankle locations
function [rl, rr] = compute_ankle_locations(q, model)
    X_footl = bodypos(model, model.idx.foot1, q);
    X_footr = bodypos(model, model.idx.foot2, q);

    rl = X_to_r(X_footl);
    rr = X_to_r(X_footr);
end

% computes ankle orientations
function [Rl, Rr] = compute_ankle_orientations(q, model)
    X_footl = bodypos(model, model.idx.foot1, q);
    X_footr = bodypos(model, model.idx.foot2, q);

    Rl = X_footl(1:3,1:3)';
    Rr = X_footr(1:3,1:3)';
end

% computes hat map of vector
function x_hat = hat(x)
    % x_hat = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    x_hat = skew(x);
end

% computes normal vectors for friction cone pyramid approximation
function nj = pyramid(n, mu)
    % mu                % coefficient of friction
    % n                 % number of sides of polygon

    % define points around base of pyramid
    angles = linspace(0,2*pi,n+1);
    angles = angles(1:end-1);
    x = mu*cos(angles); y = mu*sin(angles); z = ones(1,n);
    points = [x; y; z];
    origin = [0; 0; 0];
    
    % find normal vector to plane from 3 points
    nj = zeros(3,n);    % normal vectors returned as 3 x n matrix
    for i = 1:n
        v1 = points(:,i) - origin;
        v2 = points(:,mod(i,n)+1) - origin;
        nj(:,i) = cross(v1,v2);
        nj(:,i) = nj(:,i)/norm(nj(:,i));
    end
end

% returns inequality matrix for friction cone constraint to use in quadprog()
function nj_mat = nj_matrix(n, mu)
    nj = pyramid(n, mu);                % friction cone normal vectors
    nj_mat = zeros(n*2,3*2);            % 2 ankles
    for i = 1:2
        ind_j = i + 2*(i-1);
        ind_i = n*(i-1) + 1;
        % dot(fi,nij) = |fi| |nij| cos(theta) >= 0
            % where theta is the angle between fi and nij
            % if    theta <= 90 deg     then cos(theta) >= 0
            % if    theta  > 90 deg     then cos(theta)  < 0
        nj_mat(ind_i:ind_i+n-1, ind_j:ind_j+2) = nj';
    end
end
