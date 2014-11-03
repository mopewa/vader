global leftEncoder;
global rightEncoder;
global timeStamp;
global ranges;
global newRanges;

clc;

robot = neato('giga');
lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);
lh2 = event.listener(robot.laser, 'OnMessageReceived', ...
    @rangeDataListener);

h = figure(1);
maxIters = 10;


odomPOld = pose(0.5,0.5,pi()/2.0);


r = vaderBot(odomPOld.x, odomPOld.y, odomPOld.th, robot);

robot.startLaser();
pause(10);

endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];

localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

pose1 = pose(0.25,0.75,pi()/2.0);
pose2 = pose(0.75,0.25,0.0);
pose3 = pose(0.5,0.5,pi()/2.0);

end1 = pose(r.getPose().aToB()*pose1.bToA());
path1 = cubicSpiral.planTrajectory(end1.x, end1.y, end1.th, 1);
path1.planVelocities(.2);
[r, err] = r.executeTrajectory(path1);
err
pause(10);

end2 = pose(r.getPose().aToB()*pose2.bToA());
path2 = cubicSpiral.planTrajectory(end2.x, end2.y, end2.th, 1);
path2.planVelocities(.2);
[r, err] = r.executeTrajectory(path2);
err
pause(10);

end3 = pose(r.getPose().aToB()*pose3.bToA());
path3 = cubicSpiral.planTrajectory(end3.x, end3.y, end3.th, 1);
path3.planVelocities(.2);
[r, err] = r.executeTrajectory(path3);
err
pause(10);

finalPose = r.getPose();
finalError = sqrt((finalPose.x-0.5)^2 + (finalPose.y-0.5)^2)



