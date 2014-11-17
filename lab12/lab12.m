clc;

totalTime = tic;

robot = neato('mega');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.75*12*.0254, .75*12*.0254, pi()/2.0, robot);

robot.startLaser();
pause(3);

% create map
endpoints1 = [[0 ; 0] [0 ; 0] [4*12*0.0254 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254] [4*12*0.0254 ; 4*12*0.0254]];
localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

% create arrays of pickup and dropoff locations
pickups = [pose(1*12*.0254, 3.5*12*.0254, pi/2), ...
    pose(2*12*.0254, 3.5*12*.0254, pi/2), ...
    pose(3*12*.0254, 3.5*12*.0254, pi/2)];
dropoffs = [pose(1.75*12*.0254, .5*12*.0254, -pi/2), ...
    pose(2.25*12*.0254, .5*12*.0254, -pi/2), ...
    pose(2.75*12*.0254, .5*12*.0254, -pi/2)];

numToGet = size(pickups,2);

for i = 1:numToGet
    % localize
    ranges = robot.laser.data.ranges;
    downSample = 20;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image, 100);
    pause(0.1);
    
    % pick up object
    acquisitionPose = pose(pickups(i).x, pickups(i).y - .2, pickups(i).th);
    relPickup = pose(r.getPose().aToB()*acquisitionPose.bToA());
    pickupTarget = targetTransform2(relPickup);
    r = r.pickDropObject(robot, pickupTarget, false);
    
    % drop off object
    relDropoff = pose(r.getPose().aToB()*dropoffs(i).bToA());
    dropoffTarget = targetTransform2(relDropoff);
    r = r.pickDropObject(robot, dropoffTarget, true);
end

endtime = toc(totalTime)
