classdef controller
    %controller Implements feedback for arbitrary trajectories
    
    properties
        robotTrajectory     % the trajectory to follow
        startPose
        turnInPlace = 0; % use only kth for turn-in-place trajectory
    end
    
    properties (Constant)
        % kx = 0.3;             % x-dimension proportional control
        % ky = 1.175;             % y-dimension proportional control
        %        kx = 2;
        %        ky = 3;
        kx = .03;
        ky = 1.5;
        kth = 1;
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
            curPose = r.getPose();
            goalPose = pose(goalX,goalY,goalTheta);
            
            errorPose = pose.subtractPoses(curPose, goalPose);
            e = sqrt(errorPose.x^2+errorPose.y^2);
            % use theta-only controller for turn-in-place trajectories
            if (obj.turnInPlace) 
                errorPose.th;
                u_p = [0, errorPose.th*controller.kth];
            else 
                curTheta = curPose.th;
                
                if(isinf(curTheta) || isnan(curTheta))
                    u_p = [0; 0];
                    disp('Theta Error Skipping Controller');
                else
                    posErrorWorldFrame = [errorPose.x; errorPose.y];
                    % assumes starting theta is 0 and aligns with world frame
                    A = [cos(curTheta), -sin(curTheta); sin(curTheta), cos(curTheta)];
                    if (rank(A) < 2)
                        disp('Not Invertable');
                        A
                    end
                    transformation = A\eye(size(A));
                    posErrorRobotFrame = transformation * posErrorWorldFrame;
                    if (e <= .05)
                        control = [controller.kx, 0; 0, 0];
                    else
                        control = [controller.kx, 0; 0, controller.ky];
                    end
                    u_p = control * posErrorRobotFrame;
                end
            end
        end
    end
    
end

