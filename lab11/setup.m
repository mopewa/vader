clc;

robot = neato('giga');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.5, .5, pi()/2.0, robot);

robot.startLaser();