%% Robot prep
robot = neato('mega');
pause(3);
robot.startLaser();
pause(5);

%% Test servoDistance

% infinite running loop -- ctrc + c to end
disp('about to start');
while (true)
    ranges = robot.laser.data.ranges;
    [lVeloc, rVeloc] = servoDistance(ranges);
    disp([lVeloc, rVeloc]);
    sendVelocity(robot, lVeloc, rVeloc);
    pause(.1);
end

robot.stopLaser();
robot.close();