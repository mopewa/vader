%% robot setup

%Must Initialize Globals
global leftEncoder;
global rightEncoder;
global timeStamp;

robot = neato('centi');

lh = event.listener(robot.encoders,'OnMessageReceived', ...
    @basicEncoderListener);

pause(5);

figure(1);
hold on;

%% Part 5 Drive in a Figure 8

r = vaderBot(0, 0, 0, robot);

ks = .5;
kv = .4;
t = 12.565*ks/kv;

vl = (0.3 * kv) - 0.14125*(kv/ks)*sin(0);
vr = (0.3 * kv) + 0.14125*(kv/ks)*sin(0);

timer = tic;
currentTime = toc(timer);

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

r.drive(vl, vr);

%plot(currentTime, vl, '-.b+');
%plot(currentTime, vr, '-.bo');


while(currentTime < t)
    eL = leftEncoder - prevLeftEncoder
    eR = rightEncoder - prevRightEncoder
    dt = timeStamp - prevTimeStamp
    
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevTimeStamp = timeStamp

    r = r.updateState(eL, eR, dt);
    currentTime = toc(timer);
    vl = (0.3 * kv) - 0.14125*(kv/ks)*sin(currentTime*kv/2/ks);
    vr = (0.3 * kv) + 0.14125*(kv/ks)*sin(currentTime*kv/2/ks);
    r.drive(vl, vr);
    %plot(currentTime, vl, '-.b+');
    %plot(currentTime, vr, '-.bo');
    pause(0.01);
end

sendVelocity(robot, 0, 0);
delete(lh)

eL = leftEncoder - prevLeftEncoder
eR = rightEncoder - prevRightEncoder
dt = timeStamp - prevTimeStamp
    
prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp

r = r.updateState(eL, eR, dt);