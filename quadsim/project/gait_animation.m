swing_traj = [0.1,  0.8,  -1.5;
              0.1,  0.8,    -2;
              0.1, 0.45, -1.75;
              0.1,  0.6,  -1.5;
              0.1,  0.8,  -1.5];
tf = 0.5;               % time to take one step [s]
swing_t_steps = linspace(0, tf, size(swing_traj,1))';

L = 0.2;                % length of limb [m]

p1 = [L*cos(3*pi/2 - 0.8);
      L*sin(3*pi/2 - 0.8)];
p2 = p1 + [L*cos(3*pi/2 - (0.8 + -1.5));
           L*sin(3*pi/2 - (0.8 + -1.5))];
ground = p2(2);

t = linspace(0, 1, 50);
frames = length(t);
M(frames) = struct('cdata', [], 'colormap', []);

for i = 1:frames
    swing_joint_pos = ...
        interp1(swing_t_steps, swing_traj, mod(t(i), 0.5), 'pchip');
    t1 = swing_joint_pos(2);
    t2 = swing_joint_pos(3);
    
    p1 = [L*cos(3*pi/2 - t1);
          L*sin(3*pi/2 - t1)];
    p2 = p1 + [L*cos(3*pi/2 - (t1 + t2));
               L*sin(3*pi/2 - (t1 + t2))];

    h = figure;
    h.Visible = 'off';
    hold on
    plot([0 p1(1)], [0 p1(2)], 'linewidth', 2)
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'linewidth', 2)
    yline(ground)
    axis equal
    xlim([-0.2 0.2])
    ylim([-0.4 0])
    hold off
    M(i) = getframe(h);
end

% play animation
h.Visible = 'on';
movie(h, M, 2);
