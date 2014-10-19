clc;
hold on;

robot = neato('kilo');
pause(5);

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(0, 0, 0, robot);

robot.startLaser();
pause(5);
ranges = robot.laser.data.ranges
robot.stopLaser();

image = rangeImage(ranges, 1, 1);
[x,y,th] = image.getBestLine(12.5)
image.plotXvsY(1.5);
% image.plotRvsTh(1);

path = cubicSpiral.planTrajectory(x, y, th, 1);

path.planVelocities(.1);
path.getFinalPose()

[r, err] = r.executeTrajectory(path);

