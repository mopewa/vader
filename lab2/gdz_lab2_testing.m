%% Robot prep
robot = neato('mega');
pause(3);
robot.startLaser();
pause(10);
maxObjectRange = 2;
prevBearing = [1,0,0];
figure(1);
clf
hold on
tic;

%% Test motorFollow

% infinite running loop -- ctrc + c to end
disp('about to start');
while (true)
    ranges = robot.laser.data.ranges;
    [nearestX, nearestY, nearestTheta] = nearestObject(ranges);
    if (nearestX == maxObjectRange && nearestY == maxObjectRange && nearestTheta == 0)
        nearestX = prevBearing(1);
        nearestY = prevBearing(2);
        nearestTheta = prevBearing(3);
        i = i + 1;
    else
        prevBearing = [nearestX, nearestY, nearestTheta];
        i = 1;
    end
    if (i == 7)
       prevBearing = [1,0,0];
       i = 1;
    end
    [lVeloc, rVeloc, error] = motorFollow(nearestX, nearestY, nearestTheta);
    disp([lVeloc, rVeloc]);
    plot(toc, error, 'x');
    sendVelocity(robot, lVeloc, rVeloc);
    pause(.1);
end

robot.stopLaser();
robot.close();