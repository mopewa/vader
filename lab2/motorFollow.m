function [lVeloc, rVeloc] = motorFollow(lidar_data)
% motorFollow  Given the robot's lidar data, get a velocity to follow an
% object, keeping it at a constant distance.

    idealObjectRange = 1;
    scalingFactor = 1/2;
    W = 0.20; %Seperation between the robot wheels

    [nearestX, nearestY, nearestTheta] = nearestObject(lidar_data);
    if nearestX == idealObjectRange && nearestY == idealObjectRange && nearestTheta == 0
        % If the robot is at the ideal distance stop moving
        lVeloc = 0;
        rVeloc = 0;
    else
        % Assuming for now that robot is at sensor origin
        distance = computeDistance(0, 0, nearestX, nearestY);
        k = nearestTheta/(distance - idealObjectRange);
        V = (distance - idealObjectRange) * scalingFactor;
        lVeloc = V*(1 - (W/2)*k);
        rVeloc = V*(1 + (W/2)*k);
    end
end

function distance = computeDistance(x1, y1, x2, y2)
    distance = sqrt((x1-x2)^2 + (y1-y2)^2);
end