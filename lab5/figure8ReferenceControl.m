classdef figure8ReferenceControl
    %figure8ReferenceControl Provides the linear and angular
    % velocity required to follow a figure 8 trajectory once the time 
    % (referenced to zero at the trajectory start) is provided
    
    properties
        Ks                  % scales size of the curve
        Kv                  % scales velocity
        tPause              % amount of time to wait before and after motion 
        tFinal              % time needed for figure 8
    end

    methods
        function obj = figure8ReferenceControl(Ks, Kv, tPause)
            % Construct a figure 8 trajectory. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards. Kv scales velocity up when > 1 and Ks scales
            % the size of the curve itself down. 0.5 is a good value
            % for both.
            obj.Ks = Ks;
            obj.Kv = Kv;
            obj.tPause = tPause;
            obj.tFinal = 12.565 * (Ks / Kv);
        end
        
        function [V, w] = computeControl(obj,timeNow)
            % Return the linear and angular velocity that the robot
            % should be executing at time timeNow. Any zero velocity
            % pauses specified in the constructor are implemented here
            % too.
            if (timeNow < obj.tPause || timeNow > (obj.tFinal + obj.tPause))
                V = 0;
                w = 0;
            else
                timeNow = timeNow - obj.tPause;
                vr = 0.3 * obj.Kv + 0.14125 * (obj.Kv / obj.Ks) * ...
                    sin((timeNow * obj.Kv)/(2 * obj.Ks));
                vl = 0.3 * obj.Kv - 0.14125 * (obj.Kv / obj.Ks) * ...
                    sin((timeNow * obj.Kv)/(2 * obj.Ks));
                [V, w] = vaderBot.vlvrToVw(vl, vr);
            end
        end
        
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = obj.tFinal + (2 * obj.tPause);
        end
    end
    
end

