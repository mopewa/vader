classdef trapezoidalStepReferenceControl
    %trapezoidalStepReferenceControl Provides the linear and angular
    % velocity required to move a given distance in a straight line
    % accelerating as fast as possible to maximum velocity.
    
    properties
        amax            % maximum acceleration
        vmax            % maximum velocity
        dist            % distance to travel
        sgn             % 1 if moving forward, -1 if moving back
        tPause          % amount of time to wait before moving
        tFinal          % time needed for movement
    end
    
    methods
        function obj = trapezoidalStepReferenceControl(amax, vmax, dist, sgn, tPause)
            obj.amax = amax;
            obj.vmax = vmax;
            obj.dist = dist;
            obj.sgn = sgn;
            obj.tPause = tPause;
            obj.tFinal = (dist + vmax^2/amax)/vmax;
        end
        
        function [V, w] = computeControl(obj, timeNow)
            % Return the linear and angular velocity that the robot
            % should be executing at time timeNow. Any zero velocity
            % pauses specified in the constructor are implemented here
            % too.
            w = 0;
            if (timeNow < obj.tPause || timeNow > obj.tFinal + obj.tPause)
                V = 0;
            else
                timeNow = timeNow - obj.tPause;
                t_ramp = obj.vmax/obj.amax;
                
                if 0 < timeNow && timeNow < t_ramp
                    V = obj.sgn * obj.amax*timeNow;
                elseif 0 < obj.tFinal-timeNow && obj.tFinal-timeNow < t_ramp
                    V = obj.sgn * obj.amax * (obj.tFinal - timeNow);
                elseif t_ramp < timeNow && timeNow < obj.tFinal-t_ramp
                    V = obj.sgn * obj.vmax;
                else
                    V = 0;
                end
            end
        end
        
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and for the
            % initial and terminal pauses.
            duration = obj.tFinal + (2 * obj.tPause);
        end
    end
end

