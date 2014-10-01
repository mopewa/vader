global leftEncoder;
global rightEncoder;
global timeStamp;

robot = neato('sim');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

trapStep = trapezoidalStepReferenceControl(.75, .25, 1, 1, 1);
figure8 = figure8ReferenceControl(.5, .2, 1);

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
    
    if (~isnan(vl))

        if (vl > .3)
            vl = .3;
        elseif (vl < -.3)
            vl = -.3;
        end
        if (vr > .3)
            vr = .3;
        elseif (vr < -.3)
            vr = -.3;
        end

        r.drive(vl, vr);
    end
    
    pause(.01);
end

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

robot.shutdown();