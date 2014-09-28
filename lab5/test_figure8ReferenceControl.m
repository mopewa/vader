%% robot setup

robot = neato('sim');

% pause(5);

figure(1);
hold on;

%% Part 5 Drive in a Figure 8

ks = .5;
kv = .5;
tPause = 5;

ref_control = figure8ReferenceControl(ks, kv, tPause);

t_final = ref_control.getTrajectoryDuration();

timer = tic;
timeStamp = toc(timer);

while(timeStamp < t_final)
    timeStamp = toc(timer);
    [V, w] = ref_control.computeControl(timeStamp);
    [vl, vr] = vaderBot.VwTovlvr(V,w);
    sendVelocity(robot, vl, vr);
    pause(0.01);
end

robot.shutdown();
