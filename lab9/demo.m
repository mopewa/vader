clc;

robot = neato('femto');
h = figure(1);
maxIters = 10;

r = vaderBot(0, 0, 0, robot);

robot.startLaser();
pause(10);

robotPose = pose(15*0.0254,9*0.0254,pi()/2.0);

endpoints1 = [[0 ; 0] [0 ; 0]];
endpoints2 = [[4*12*0.0254 ; 0] [0 ; 4*12*0.0254]];

localizer = lineMapLocalizer(endpoints1, endpoints2, 0.1, 0.001, 0.0005);

downSample = 10;

driver = robotKeypressDriver(robot, gcf);

while (true)
    ranges = robot.laser.data.ranges;
    image = rangeImage(ranges, downSample, false);

    [success, outPose] = refinePose(localizer,robotPose,[image.xArray; image.yArray; ones(1, 360/downSample)],maxIters);
    if (~success)
       disp('localized!'); 
    else 
        disp('fill iter');
    end
    robotPose = outPose;
    pause(0.1);
    
end

