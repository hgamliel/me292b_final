    % qroll = qroll;            % torso roll
    % q2st = q2R;                 % stance hip yaws
    % qpitch = qpitch;          % torso pitch
    % qLLst = sqrt(0.5292*cos(q4R + 0.035) + 0.5301);
    % qLRsw = qroll + q1L;
    % q2sw = q2L;                 % swing hip yaw
    % qLPsw = -qpitch + q3L - acos( 0.5*(cos(q4L+0.035)+0.5292) / sqrt(0.5292*cos(q4L+0.035)+0.5301)) + 0.1;
    % qLLsw = sqrt(0.5292*cos(q4L + 0.035) + 0.5301);
    % qFPsw = -qpitch + q7L + 1.1;
    
    % f and g vectors
    % [H,Cq_G_JTF] = HandC( model, q, qd, f_ext );
    % f_ext???? m(v - v_prev)/dt, I(omega - omega_prev)/dt

    zero = zeros(NDof,1);
    f = [dq; inv(D)*(-C*dq-G)] + [zero; inv(D)*JSt'*FSt_no_u];
    zero = zeros(NDof,length(u));
    g = [zero; inv(D)*B] + [zero; inv(D)*JSt'*FSt_u];

    % virtual constraint 
    x0 = getInitialState(model); rhip0 = x0(1:3);
    y = s(1:6) - [rhip0; zeros(3,1)];
    dy = s(21:27);

    % PD on virtual constraints?, refer to paper 1

    % [mass com_offset] = mcI_inv(model, j) ;
    % mass_sum = mass_sum + mass ;
    % body_COM_jac = bodyJac_vel(model,j,q,xlt(com_offset)); 

    % Lie derivatives
    dy_dq = jacobian(y, q);
    d2y__ = [jacobian(dy_dq*dq, q), dy_dq];

    Lfy = dy_dq*dq;
    Lgy = zeros(size(Lfy,1),length(u));
    Lf2y = d2y__*f;
    LgLfy = d2y__*g;

    % input-output linearizing controller
    a = 0.9;
    E = 0.1;
    phi_a = @(x1,x2) x1 + 1/(2-a) * sign(x2) * abs(x2)^(2-a);
    psi_a = @(x1,x2) -sign(x2)*abs(x2)^a - sign(phi_a(x1,x2))*abs(phi_a(x1,x2))^(a/(2-a));

    v = (1/E^2)*[psi_a(y(1),E*dy(1)); psi_a(y(2),E*dy(2))];
    u = inv(LgLfy)*(-Lf2y + v);

% calculates moment of inertia about Cassie COM
function I = I_COM(s, model)
    I_COM = zeros(3);
    for i = 1:model.NB
        [mass, com_offset, I] = mcI_inv(model,i);
        R = com_offset;
        if(mass(i) > 0)
            I_COM = I + mass*(norm(R)^2*eye(3) - R*R');
        end
    end
end

% torque limit
tauMax = repmat([25*4.5; 25*4.5; 16*12.2; 16*12.2; 50*0.9], 2, 1);
tauComp = max(tau./tauMax);
tau = tau/tauComp;
