classdef controller
    %controller Implements feedback for arbitrary trajectories
    
    properties
        robotTrajectory     % the trajectory to follow
        startPose
    end
    
    properties (Constant)
        % kx = 0.3;             % x-dimension proportional control
        % ky = 1.175;             % y-dimension proportional control
        %        kx = 2;
        %        ky = 3;
        kx = .25;
        ky = 1.5;
    end
    
    methods
        function obj = controller(robotTrajectory, startPose)
            obj.robotTrajectory = robotTrajectory;
            obj.startPose = startPose;
        end
        
        function [u_p, e] = getVelocity(obj, t, r)
            % compute the feedback for a robot r that should be
            % following robotTrajectory.
            % Currently, this uses a three degree of freedom linear controller.
            [goalX, goalY, goalTheta] = obj.robotTrajectory.getPoseAtTime(t);
            curX = r.xPos(r.index);
            curY = r.yPos(r.index);
            curTheta = r.theta(r.index);
            
            %{
            sX = obj.startPose(1);
            sY = obj.startPose(2);
            sTheta = obj.startPose(3);

            GToS = [cos(goalTheta), -sin(goalTheta), goalX; sin(goalTheta), cos(goalTheta), goalY; 0 0 1];
            RToS = [cos(curTheta), -sin(curTheta), curX; sin(curTheta), cos(curTheta), curY; 0 0 1];
            SToW = [cos(sTheta), -sin(sTheta), sX; sin(sTheta), cos(sTheta), sY; 0 0 1];
            
            GToW = SToW * GToS;
            RToW = SToW * RToS;
            
            goalX = GToW(1,3);
            goalY = GToW(2,3);
            goalTheta = atan2(GToW(2,1), GToW(1,1));
            
            curX = RToW(1,3);
            curY = RToW(2,3);
            curTheta = atan2(RToW(2,1), RToW(1,1));
            %}
            
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

