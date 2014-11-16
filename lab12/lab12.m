clc;

robot = neato('femto');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.5, .5, pi()/2.0, robot);

%robot.startLaser();
pause(3);

% create map
endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];
localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

for i = 1:1
    ranges = robot.laser.data.ranges;
    downSample = 10;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image, 100);
    pause(0.1);

    % find object in range image
    foundLine = 0;
    while (~foundLine)
        ranges = robot.laser.data.ranges;
        image = rangeImage(ranges, 1, 1);
        [x,y,th] = image.findObject(.14); 
        foundLine = x || y;
        pause(.1);
    end

    % get object pose, if large angle difference, turn in place
    [xNew, yNew, thNew] = targetTransform(x,y,th)
    
    newPose = pose(xNew, yNew, thNew);
    
    r.pickDropObject(robot, newPose, false);
end