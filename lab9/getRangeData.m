function [ ranges ] = getRangeData()
    robot = neato('milli');
    robot.startLaser();
    pause(10);
    ranges = robot.laser.data.ranges;
    robot.stopLaser();
end

