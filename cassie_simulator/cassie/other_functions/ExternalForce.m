function f_ext = ExternalForce(t, q,model)
% F_pert 6x1 - roll, pitch, yaw, x,y,z
F_pert =       7.5*[1 -1 1            -1 1 -1]';
F_pert =       0*[1 -1 1            -1 1 -1]';

yaw_max = model.M*9.81*.1 ;
y_max = model.M*9.81*.5;

if t<.1
    F_pert = [0 0 0 0 y_max 0]';
else
    F_pert = zeros(6,1);
end

% apply perturbation force on torso
f_ext = cell(1,model.NB);
bXw_curr = bodypos(model, model.idx.torso, q) ;
f_ext{model.idx.torso} = bXw_curr' * F_pert;