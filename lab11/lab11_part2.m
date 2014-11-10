
% Done in setup.m so we dont have to start our code with a 10s pause

clc;

robot = neato('giga');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.5, .5, pi()/2.0, robot);

robot.startLaser();
pause(10);

% create map
endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];
localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

% 5 seconds of gradient descent to acurately localize
% timer = tic;
% while(toc(timer) < 5)
%     ranges = robot.laser.data.ranges;
%     downSample = 10;
%     image = rangeImage(ranges, downSample, false);
%     [r, success] = r.processRangeImage(image);
%     pause(0.2);
% end
ranges = robot.laser.data.ranges;
downSample = 10;
image = rangeImage(ranges, downSample, false);
[r, success] = r.processRangeImage(image);
pause(0.1);

% find object in range image
foundLine = 0;
while (~foundLine)
    ranges = robot.laser.data.ranges;
    image = rangeImage(ranges, 1, 1);
    [x,y,th] = image.findObject(.14); % not sure where this number comes from, taken from lab 8
    foundLine = x || y;
end

% go to object
[xNew, yNew, thNew] = targetTransform(x,y,th)
newPose = pose(xNew, yNew, thNew);
[r, error] = r.executeTrajectoryToRelativePose(newPose, 1);

finalPose = r.getPose();
finalPose.x
finalPose.y
finalPose.th

% back up 15 centimeters, then turn 180 degrees
r.moveRelDistance(-.15);
r.moveRelAngle(pi);