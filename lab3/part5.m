%% robot setup

robot = neato('centi');

pause(5);

figure(1);
hold on;
lh = event.listener(robot.encoders,'OnMessageReceived', ...
    @neatoEncoderEventListener);

%% Part 5 Drive in a Figure 8

ks = .5;
kv = .4;
t = 12.565*ks/kv;

vl = (0.3 * kv) - 0.14125*(kv/ks)*sin(0);
vr = (0.3 * kv) + 0.14125*(kv/ks)*sin(0);

timer = tic;
timeStamp = toc(timer);
sendVelocity(robot, vl, vr);

plot(timeStamp, vl, '-.b+');
plot(timeStamp, vr, '-.bo');

while(timeStamp < t)
    timeStamp = toc(timer);
    vl = (0.3 * kv) - 0.14125*(kv/ks)*sin(timeStamp*kv/2/ks);
    vr = (0.3 * kv) + 0.14125*(kv/ks)*sin(timeStamp*kv/2/ks);
    sendVelocity(robot, vl, vr);
    plot(timeStamp, vl, '-.b+');
    plot(timeStamp, vr, '-.bo');
    pause(0.01);
end

sendVelocity(robot, 0, 0);
delete(lh)