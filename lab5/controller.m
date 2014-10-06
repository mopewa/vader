classdef controller
    %controller Implements feedback for arbitrary trajectories
    
    properties
        robotTrajectory     % the trajectory to follow
    end
    
    properties (Constant)
       % kx = 0.3;             % x-dimension proportional control
       % ky = 1.175;             % y-dimension proportional control  
%        kx = 2;
%        ky = 3;
         kx = 1;
         ky = 5.75;
    end
    
    methods
        function obj = controller(robotTrajectory)
            obj.robotTrajectory = robotTrajectory;
        end
        
        function [u_p, e] = getVelocity(obj, t, r)
            % compute the feedback for a robot r that should be
            % following robotTrajectory.
            % Currently, this uses a three degree of freedom linear controller. 
            [goalX, goalY, goalTheta] = obj.robotTrajectory.getPoseAtTime(t);
            curX = r.xPos(r.index);
            curY = r.yPos(r.index);
            curTheta = r.theta(r.index);
            posErrorWorldFrame = [goalX - curX; goalY - curY];
            % assumes starting theta is 0 and aligns with world frame
            transformation = inv([cos(curTheta), -sin(curTheta); sin(curTheta), cos(curTheta)]);
            posErrorRobotFrame = transformation * posErrorWorldFrame;
            e = sqrt((goalX - curX)^2 + (goalY - curY)^2);
            if (e <= .05)
                control = [controller.kx, 0; 0, 0];
            else
                control = [controller.kx, 0; 0, controller.ky];
            end
            u_p = control * posErrorRobotFrame;
        end
    end
    
end

