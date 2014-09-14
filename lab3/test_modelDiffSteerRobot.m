
%% Cornu Spiral
k = 2;

t0 = 0;
tf = 11.207 * k;
dt = .001;
numSteps = (tf-t0)/dt;

vr = zeros(numSteps+1, 1);
vl = zeros(numSteps+1, 1);

for i = 1:numSteps
    vr(i+1) = 1000 * (.1/k + .01174*(i*dt)/k^2);
    vl(i+1) = 1000 * (.1/k - .01174*(i*dt)/k^2);
end

[x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt);

%% Figure 8
% (Need to adjust plot limits in modelDiffSteerRobot for correct graph)
% ks = .5;
% kv = .4;
% 
% t0 = 0;
% tf = 12.565 * ks/kv;
% dt = .001;
% numSteps = (tf-t0)/dt
% 
% vr = zeros(ceil(numSteps+1), 1);
% vl = zeros(ceil(numSteps+1), 1);
% 
% for i = 1:numSteps
%     vr(i+1) = 1000 * (.3*kv + .14125 * kv/ks * sin(i*dt*kv/(2*ks)));
%     vl(i+1) = 1000 * (.3*kv - .14125 * kv/ks * sin(i*dt*kv/(2*ks)));
% end
% 
% [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt);