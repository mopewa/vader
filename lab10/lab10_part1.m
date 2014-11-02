global leftEncoder;
global rightEncoder;
global timeStamp;

clc;

robot = neato('centi');
lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

h = figure(1);
maxIters = 10;


odomPOld = pose(15*0.0254,9*0.0254,pi()/2.0);


r = vaderBot(odomPOld.x, odomPOld.y + .09, odomPOld.th, robot);

robot.startLaser();
pause(10);

endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];

localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);

downSample = 10;

driver = robotKeypressDriver(robot, gcf);

leftEncoderStart = leftEncoder;
rightEncoderStart = rightEncoder;

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

poseFused = odomPOld;

while (true)
    eL = leftEncoder - prevLeftEncoder;
    eR = rightEncoder - prevRightEncoder;
    dt = timeStamp - prevTimeStamp;
    
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevTimeStamp = timeStamp;
    
    r = r.updateState(eL, eR, dt); 
    
    odomPNew = pose(r.xPos(r.index), r.yPos(r.index), r.theta(r.index));
    
    deltaOdom = pose.subtractPoses(odomPOld, odomPNew);
    
    odomPOld = odomPNew;
    
    poseFused = pose.addPoses(poseFused, deltaOdom);
    
    ranges = robot.laser.data.ranges;
    image = rangeImage(ranges, downSample, false);

    [success, poseMap] = refinePose(localizer,poseFused,[image.xArray; image.yArray; ones(1, 360/downSample)],maxIters);

    subPose = pose.subtractPoses(poseFused, poseMap);
    fractionPose = pose(.25 * subPose.x, .25*subPose.y, .25*subPose.th);
    poseFused = pose.addPoses(poseFused, fractionPose);
    
    r = r.setPose(poseFused);
    
    if (success)
        wp = poseFused.bToA()*[image.xArray; image.yArray; ones(1, 360/downSample)];
        plot(wp(1, :), wp(2, :), 'og');hold on;
    end
     poseFused.x
     poseFused.y
     poseFused.th
    pause(1.5);
    
end

