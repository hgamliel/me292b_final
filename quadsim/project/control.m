function [out,use_torque,kp,kd] = control(t,v,omega,quat,q,dq,foot_contact)
    %%%%%% Inputs %%%%%%
    %%% t %%%
    % Time elapsed in the simulation [s]
    % Float type

    %%% v %%%
    % Robot base linear velocity [m/s]
    % [x, y, z], dim=3

    %%% omega %%%
    % Robot base angular velocity [rad/s]
    % [x, y, z], dim=3

    %%% quat %%%
    % Robot base orientation in quaternion [-]
    % [x, y, z, w], dim=4

    %%% q %%%
    % Joint position [rad]
    % [front_right_hip, front_right_thigh, front_right_calf,
    %   front_left_hip,  front_left_thigh,  front_left_calf,
    %   rear_right_hip,  rear_right_thigh,  rear_right_calf,
    %    rear_left_hip,   rear_left_thigh,   rear_left_calf]
    % dim=12

    %%% dq %%%
    % Joint velocity [rad/s]
    % Order and dimension same as q

    %%% foot_contact %%%
    % Robot foot contact [-] (1 stands for contact, 0 non-contact)
    % [front_right, front_left, rear_right, rear_left], dim=4


    %%%%%% Outputs %%%%%%
    %%% out %%%
    % Output (joint torque [Nm] or position [rad])
    % Order and dimension same as q

    %%% use_torque %%%
    % If use torque output (otherwise use position)

    %%% kp %%%
    % Desired joint kp values (valid only if use position control)
    % Order and dimension same as q

    %%% kd %%%
    % Desired joint kd values (valid only if use position control)
    % Order and dimension same as q


    %%%%%% Examples %%%%%%
    %%% Position control example %%%
    % Create a dummy output with zero positions
    out = [0 0 0 0 0 0 0 0 0 0 0 0];
    % Set to use position control
    use_torque = false;
    % Create a dummy kp with zeros
    kp = [0 0 0 0 0 0 0 0 0 0 0 0];
    % Create a dummy kd with zeros
    kd = [0 0 0 0 0 0 0 0 0 0 0 0];

    %%% Torque control example %%%
    % Create a dummy output with zero torques
    out = [0 0 0 0 0 0 0 0 0 0 0 0];
    % Set to use torque control
    use_torque = true;
    % Create a placeholder for kp
    kp = [0 0 0 0 0 0 0 0 0 0 0 0];
    % Create a placeholder for kd
    kd = [0 0 0 0 0 0 0 0 0 0 0 0];
end
