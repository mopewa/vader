global leftEncoder;
global rightEncoder;
global timeStamp;
global ranges;
global newRanges;

clc;

robot = neato('centi');
lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);
lh2 = event.listener(robot.laser, 'OnMessageReceived', ...
    @rangeDataListener);

h = figure(1);
maxIters = 10;


odomPOld = pose(15*0.0254,9*0.0254,pi()/2.0);


r = vaderBot(odomPOld.x, odomPOld.y + .09, odomPOld.th, robot);

robot.startLaser();
pause(10);

endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];

localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);
r = r.setLocalizer(localizer);

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
    
    r = r.processOdometryData(eL, eR, dt); 
    
    % process range data when new range data arrives
    if (newRanges)
        newRanges = 0;
        image = rangeImage(ranges, downSample, false);
        [r, success] = r.processRangeImage(image);

        if (success)
            wp = r.getPose().bToA()*[image.xArray; image.yArray; ones(1, 360/downSample)];
            plot(wp(1, :), wp(2, :), 'og');hold on;
        end
    end
    pause(.005);
end

