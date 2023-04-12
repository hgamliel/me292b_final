function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
F_pert =       [0 0 0            0 0 0]';

m = model.M;            % robot mass [kg]
g = 9.81;               % acceleration due to gravity [m/s^2]

% impulses of duration 0.1s or less, applied at t = 0, with:
% forces of magnitude 50% of M*g or lower along translational directions
% and 10% of M*g or lower along orientation directions
f_max = 0.5*m*g;
tau_max = 0.1*m*g;

if (t > 0.1)
    F_pert = zeros(6,1);
else
    F_pert = [0 0 0 0 0.3*f_max 0]';              
end

% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;