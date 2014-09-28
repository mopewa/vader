classdef trajectoryFollower
    %trajectoryFollower Uses feedforward and feedback to follow an
    % arbitrary trajectory.
    
    properties
        robotTrajectory             % the trajectory to follow
        controller                  % provides feedback for the trajectory
    end
    
    methods
        function obj = trajectoryFollower(robotTrajectory)
            obj.robotTrajectory = robotTrajectory;
            obj.controller = controller(robotTrajectory);
        end
        
        function [vl, vr] = getVelocity(obj, t, vaderBot)
            % get feedforward velocity
            [V, w] = obj.robotTrajectory.getVelocityAtTime(t);
            
            % get feedback adjustment
            u_p = obj.controller.getVelocity(t, vaderBot);
            V = V + u_p(1);
            w = w + u_p(2);
            
            [vl, vr] = vaderBot.VwTovlvr(V, w);
        end
    end
end

