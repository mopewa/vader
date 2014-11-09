classdef trajectoryFollower
    %trajectoryFollower Uses feedforward and feedback to follow an
    % arbitrary trajectory.
    
    properties
        robotTrajectory             % the trajectory to follow
        feedbackController          % provides feedback for the trajectory
        logV                        % Provides an array for logging velocity values
        logW                        % Provides an array for logging angular velocity values
        error                       % Provides an array for logging trajectory error
        time                        % Provides an array for logging timestamps
        feedback                    % Switch to turn on and off feedback
        startPose
    end
    
    methods
        function obj = trajectoryFollower(robotTrajectory, startPose)
            obj.robotTrajectory = robotTrajectory;
            obj.feedbackController = controller(robotTrajectory, startPose);
            obj.startPose = startPose;
            obj.feedback = true;
        end
        
        function obj = setTurnInPlace(obj, turnInPlace)
            obj.feedbackController.turnInPlace = turnInPlace;
        end
        
        function [vl, vr, obj] = getVelocity(obj, t, vaderBot)
            % get feedforward velocity
            [V, w] = obj.robotTrajectory.getVelocityAtTime(t);
            
            % get feedback adjustment
            if (obj.feedback)
                [u_p, e] = obj.feedbackController.getVelocity(t, vaderBot);
                V = V + u_p(1);
                w = w + u_p(2);
            end
            
            obj.logV = [obj.logV V];
            obj.logW = [obj.logW w];
            obj.time = [obj.time t];
            obj.error = [obj.error e];
            
            [vl, vr] = vaderBot.VwTovlvr(V, w);
        end
    end
end

