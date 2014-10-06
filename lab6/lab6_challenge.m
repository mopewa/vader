clc;
hold on;

robot = neato('kilo');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(0, 0, 0, robot);

path1 = cubicSpiral.planTrajectory(0.25, 0.25, 0.0, 1);
path1.planVelocities(.2);

r.executeTrajectory(path1);
pause(10);

path2 = cubicSpiral.planTrajectory(-0.5, -0.5, -pi/2.0, 1);
path2.planVelocities(.2);

r.executeTrajectory(path2);
pause(10);

path3 = cubicSpiral.planTrajectory(-0.25, 0.25, pi/2.0, 1);
path3.planVelocities(.2);

r.executeTrajectory(path3);
pause(10);


