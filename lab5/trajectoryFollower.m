classdef trajectoryFollower
    %trajectoryFollower Uses feedforward and feedback to follow an
    % arbitrary trajectory.
    
    properties
        robotTrajectory             % the trajectory to follow
        controller                  % provides feedback for the trajectory
        logV                        % Provides an array for logging velocity values
        logW                        % Provides an array for logging angular velocity values
        error                       % Provides an array for logging trajectory error
        time                        % Provides an array for logging timestamps
    end
    
    methods
        function obj = trajectoryFollower(robotTrajectory)
            obj.robotTrajectory = robotTrajectory;
            obj.controller = controller(robotTrajectory);
        end
        
        function [vl, vr, obj] = getVelocity(obj, t, vaderBot)
            % get feedforward velocity
            [V, w] = obj.robotTrajectory.getVelocityAtTime(t);
            
            % get feedback adjustment
            [u_p, e] = obj.controller.getVelocity(t, vaderBot);
            V = V + u_p(1);
            w = w + u_p(2);
            
            obj.logV = [obj.logV V];
            obj.logW = [obj.logW w];
            obj.time = [obj.time t];
            obj.error = [obj.error e];
            
            [vl, vr] = vaderBot.VwTovlvr(V, w);
        end
    end
end

