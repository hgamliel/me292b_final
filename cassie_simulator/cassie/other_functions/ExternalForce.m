function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
F_pert =       7.5*[1 -1 1            -1 1 -1]';
% max. x, y, z perturbation = 25 N
% max. r, p, y perturbation = 37 N*m

% impulses of duration 0.1s or lesser with:
% forces of magnitude 50% of M*g or lower along translational directions
% and 10% of M*g or lower along orientation directions
m = model.M;            % robot mass [kg]
g = 9.81;               % acceleration due to gravity [m/s^2]
if (t > 0.1)
    F_pert = zeros(6,1);
else
    F_pert = [0 0 0 0.5*m*g 0 0]';              
end

% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;