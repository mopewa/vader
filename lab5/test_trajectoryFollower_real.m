global leftEncoder;
global rightEncoder;
global timeStamp;

clc;
hold on;

robot = neato('kilo');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

trapStep = trapezoidalStepReferenceControl(.75, .25, 1, 1, 1);
figure8 = figure8ReferenceControl(.5, .2, 5);

referenceControl = figure8;
traj = robotTrajectory(referenceControl, [0,0,0], 0);
follower = trajectoryFollower(traj);

r = vaderBot(0, 0, 0, robot);

vl = 0;
vr = 0;

leftEncoderStart = leftEncoder;
rightEncoderStart = rightEncoder;

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

timer = tic;
currentTime = toc(timer);

while (currentTime < referenceControl.getTrajectoryDuration())
%     currentTime
    eL = leftEncoder - prevLeftEncoder;
    eR = rightEncoder - prevRightEncoder;
    dt = timeStamp - prevTimeStamp;
    
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevTimeStamp = timeStamp;
    
    r = r.updateState(eL, eR, dt);    
    
    currentTime = toc(timer);
    [vl, vr, follower] = follower.getVelocity(currentTime, r);
    
%     if (vl > .3)
%         vl = .3;
%         vr = vr * .3/vl;
%     end
%     if (vr > .3)
%         vr = .3;
%         vl = vl * .3/vr;
%     end
    
    r.drive(vl, vr);
    
    pause(.005);
end

eL = leftEncoder - prevLeftEncoder;
eR = rightEncoder - prevRightEncoder;
dt = timeStamp - prevTimeStamp;
    
prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;
    
r = r.updateState(eL, eR, dt);    
    
currentTime = toc(timer);
[vl, vr, follower] = follower.getVelocity(currentTime, r);
    
r.drive(vl, vr);
    
pause(1);

sendVelocity(robot, 0, 0);
delete(lh);

eL = leftEncoder - prevLeftEncoder;
eR = rightEncoder - prevRightEncoder;
dt = timeStamp - prevTimeStamp;

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

r = r.updateState(eL, eR, dt);

disp([r.xPos(r.index), r.yPos(r.index)]);

figure(1);
plot(r.xPos, r.yPos, traj.poses(:,1)', traj.poses(:,2)');
figure(2);
plot(follower.time, follower.error, '-r');
