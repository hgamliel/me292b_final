function J = calcScore(t, s, model)

q = s(1 : model.n, :);
dq = s(model.n+1 : 2*model.n, :);
v = dq(1:3, :);
w = dq(4:6, :);

vNorm = vecnorm(v);
wNorm = vecnorm(w);

vInfo = lsiminfo(vNorm, t);
wInfo = lsiminfo(wNorm, t);
tSettle = vInfo.SettlingTime;
vNormEnd = norm(v(:, end));
wNormEnd = norm(w(:, end));

vMax = 5;
wMax = 1;
tMax = 5;

if t(end) < 5
    n = round(length(t)*0.8);
else
    n = length(t);
end

figure
hold on
plot(t(1:n),v(1,1:n))
plot(t(1:n),v(2,1:n))
plot(t(1:n),v(3,1:n))
plot(t(1:n),vNorm(1:n))
xlabel('time (s)')
ylabel('m/s')
title('Linear Velocity')
legend('v_x', 'v_y', 'v_z', '||v||')

figure
hold on
plot(t(1:n),w(1,1:n))
plot(t(1:n),w(2,1:n))
plot(t(1:n),w(3,1:n))
plot(t(1:n),wNorm(1:n))
xlabel('time (s)')
ylabel('rad/s')
title('Angular Velocity')
legend('d yaw', 'd pitch', 'd roll', '||\omega||')



if t < 5
    J = 0; % Robot has fallen down
else
    J = 100*(max(0,(vMax-vNormEnd)) + max(0, (wMax - wNormEnd)) + (tMax - tSettle))/(vMax + wMax + tMax);
end


