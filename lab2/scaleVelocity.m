function [lVeloc, rVeloc] = scaleVelocity(lVeloc, rVeloc)
% scaleVelocity  Given the robot's left and right motor speeds, scales
% the velocities to fit within the max speed of the robot.

    maxV = 0.3;
    currentMax = max([abs(lVeloc) abs(rVeloc)]);

    if (currentMax > maxV) 
        lVeloc = (lVeloc/currentMax)*maxV;
        rVeloc = (rVeloc/currentMax)*maxV;
    end
    
end