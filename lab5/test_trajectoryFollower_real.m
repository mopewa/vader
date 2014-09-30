global leftEncoder;
global rightEncoder;
global timeStamp;

clc;
hold on;

robot = neato('centi');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

trapStep = trapezoidalStepReferenceControl(.75, .25, 1, 1, 1);
figure8 = figure8ReferenceControl(.8, .2, 5);

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

while (currentTime < referenceControl.getTrajectoryDuration() + 1)
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
%     end
%     if (vr > .3)
%         vr = .3;
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
    
% currentTime = toc(timer);
% [vl, vr, follower] = follower.getVelocity(currentTime, r);
%     
% r.drive(vr, vl);
%     
% pause(1);

sendVelocity(robot, 0, 0);
delete(lh);

eL = leftEncoder - prevLeftEncoder;
eR = rightEncoder - prevRightEncoder;
dt = timeStamp - prevTimeStamp;

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

r = r.updateState(eL, eR, dt);

[finalX, finalY, finalTheta] = traj.getPoseAtTime(currentTime)
xError = r.xPos(r.index)-finalX
yError = r.yPos(r.index)-finalY
totalError = sqrt(xError^2 + yError^2)

figure(1);
plot(r.xPos, r.yPos, traj.poses(:,1)', traj.poses(:,2)');
figure(2);
plot(follower.time, follower.error, '-r');
