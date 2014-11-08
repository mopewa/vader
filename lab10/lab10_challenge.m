clc;
hold on;

robot = neato('milli');
totalError = 0;

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(0.5, 0.5, pi()/2.0, robot);

pose1 = pose(0.25,0.75,pi()/2.0);
pose2 = pose(0.75,0.25,0.0);
pose3 = pose(0.5,0.5,pi()/2.0);

 robot.startLaser();
 pause(10);
 
 endpoints1 = [[0 ; 0] [0 ; 0]];
 endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];
 
 localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
 r = r.setLocalizer(localizer);

 ranges = robot.laser.data.ranges;
 downSample = 10;
 image = rangeImage(ranges, downSample, false);
[r, success] = r.processRangeImage(image);

end1 = pose(r.getPose().aToB()*pose1.bToA());
path1 = cubicSpiral.planTrajectory(end1.x, end1.y, end1.th, 1);
path1.planVelocities(.2);
 
[r, err] = r.executeTrajectory(path1, true);
pause(1);

timer = tic;
while(toc(timer) < 5)
    ranges = robot.laser.data.ranges;
    downSample = 10;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image);
end

rPose = r.getPose();
Error1 = sqrt((rPose.x - 0.25)^2 + (rPose.y - 0.75)^2)


end2 = pose(r.getPose().aToB()*pose2.bToA());
path2 = cubicSpiral.planTrajectory(end2.x, end2.y, end2.th, 1);
path2.planVelocities(.2);

[r, err] = r.executeTrajectory(path2, true);
pause(1);

timer = tic;
while(toc(timer) < 5)
    ranges = robot.laser.data.ranges;
    downSample = 10;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image);
end
rPose = r.getPose();
Error2 = sqrt((rPose.x - 0.75)^2 + (rPose.y - 0.25)^2)

end3 = pose(r.getPose().aToB()*pose3.bToA());
path3 = cubicSpiral.planTrajectory(end3.x, end3.y, end3.th, 1);
path3.planVelocities(.2);

[r, err] = r.executeTrajectory(path3, true);

timer = tic;
while(toc(timer) < 5)
    ranges = robot.laser.data.ranges;
    downSample = 10;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image);
end

rPose = r.getPose();
Error3 = sqrt((rPose.x - 0.5)^2 + (rPose.y - 0.5)^2)

