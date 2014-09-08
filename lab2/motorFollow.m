function [lVeloc, rVeloc, error] = motorFollow(x, y, theta)
% motorFollow  Given the robot's lidar data, get a velocity to follow an
% object, keeping it at a constant distance.

    idealObjectRange = 1;
    scalingFactor = 0.5;
    W = 0.235; %Seperation between the robot wheels

    distance = computeDistance(0, 0, x, y);
    if distance == idealObjectRange && theta == 0
        % If the robot is at the ideal distance stop moving
        lVeloc = 0;
        rVeloc = 0;
        error = 0;
    else
        % Assuming for now that robot is at sensor origin
        theta = (theta/360)*2*pi;
        V = (distance - idealObjectRange);
        lVeloc = (V - (W/2)*theta)*scalingFactor;
        rVeloc = (V + (W/2)*theta)*scalingFactor;
        [lVeloc, rVeloc] = scaleVelocity(lVeloc, rVeloc);
        error = (distance - idealObjectRange);
    end
end

function distance = computeDistance(x1, y1, x2, y2)
    distance = sqrt((x1-x2)^2 + (y1-y2)^2);
end