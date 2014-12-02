clc;

totalTime = tic;

robot = neato('nano');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.75*12*.0254, .75*12*.0254, -pi()/2.0, robot);

robot.startLaser();
pause(3);

% create map
endpoints1 = [[0 ; 0] [0 ; 0] [8*12*0.0254 ; 0]];
endpoints2 = [[8*12*0.0254 ; 0] [0 ; 6*12*0.0254] [8*12*0.0254 ; 6*12*0.0254]];
localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

% create arrays of pickup and dropoff locations
acquisitionOffset = .25;
pickups = [pose(4*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(3*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(2*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(1*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(7*12*.0254-acquisitionOffset, 2*12*.0254, 0), ...
    pose(7*12*.0254-acquisitionOffset, 3*12*.0254, 0), ...
    pose(7*12*.0254-acquisitionOffset, 4*12*.0254, 0), ...
    pose(5*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(6*12*.0254, 6*12*.0254-acquisitionOffset, pi/2), ...
    pose(7*12*.0254, 6*12*.0254-acquisitionOffset, pi/2)];
dropoffs = [pose(1*12*.0254, 1*12*.0254, -pi/2), ...
    pose(2*12*.0254, .8*12*.0254, -pi/2), ...
    pose(3*12*.0254, .8*12*.0254, -pi/2), ...
    pose(4*12*.0254, .8*12*.0254, -pi/2), ...
    pose(5*12*.0254, .8*12*.0254, -pi/2), ...
    pose(6*12*.0254, .8*12*.0254, -pi/2), ...
    pose(7*12*.0254, .8*12*.0254, -pi/2)];

numToGet = size(pickups,2);

r = r.moveRelAngle(3*pi/4);

dropoffIndex = 1;

robot.forksDown();
for pickupIndex = 1:numToGet
    % localize
    ranges = robot.laser.data.ranges;
    downSample = 20;
    image = rangeImage(ranges, downSample, false);
    [r, success] = r.processRangeImage(image, 100);
    pause(0.1);
    
    % pick up object
    relPickup = pose(r.getPose().aToB()*pickups(pickupIndex).bToA());
    pickupTarget = targetTransform2(relPickup);
    [r, pickedUp] = r.pickDropObject(robot, pickupTarget, false, dropoffs(dropoffIndex));
    
    if (pickedUp)
        % drop off object
        nextPose = dropoffs(dropoffIndex);
        if (pickupIndex < numToGet)
            nextPose = pickups(pickupIndex+1);
        end
        relDropoff = pose(r.getPose().aToB()*dropoffs(dropoffIndex).bToA());
        dropoffTarget = targetTransform2(relDropoff);
        r = r.pickDropObject(robot, dropoffTarget, true, nextPose);
        if (dropoffIndex == 7)
            break;
        end
        dropoffIndex = dropoffIndex+1;
    else
        % if object was missing, back up to make finding the next one
        % easier
        r = r.moveRelDistance(-.5);
    end
    curTime = toc(totalTime)
end

endtime = toc(totalTime)
