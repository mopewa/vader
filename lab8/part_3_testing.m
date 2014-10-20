clc;

robot = neato('femto');
pause(5);

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(0, 0, 0, robot);

foundLine = 0;
robot.startLaser();
pause(5);

while (~foundLine)
    ranges = robot.laser.data.ranges;

    image = rangeImage(ranges, 1, 1);
    [x,y,th] = image.getBestLine(.5125)
    foundLine = x || y;
end
robot.stopLaser();
% image.plotXvsY(1.5); 
% image.plotRvsTh(1);

if (x == 0 && y == 0)
    disp('Did not see a line');
else 

    [xNew, yNew, thNew] = targetTransform(x,y, th)

    path = cubicSpiral.planTrajectory(xNew, yNew, thNew, 1);

    path.planVelocities(.2);
    path.getFinalPose()

    [r, err] = r.executeTrajectory(path);
end

