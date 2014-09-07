function [lVeloc, rVeloc] = servoDistance(lidar_data)
% servoDistance  Given the robot's lidar data, get a velocity to follow an
% object in a straight line, keeping it at a constant distance.

    idealObjectRange = 1;
    scalingFactor = 1/2;

    [nearestX, nearestY, nearestTheta] = nearestObject(lidar_data);
    if nearestX == idealObjectRange && nearestY == idealObjectRange && nearestTheta == 0
        lVeloc = 0;
        rVeloc = 0;
    else
        % Assuming for now that robot is at sensor origin
        distance = computeDistance(0, 0, nearestX, nearestY);
        lVeloc = (distance-idealObjectRange) * scalingFactor;
        rVeloc = (distance-idealObjectRange) * scalingFactor;
    end
end

function distance = computeDistance(x1, y1, x2, y2)
    distance = sqrt((x1-x2)^2 + (y1-y2)^2);
end

