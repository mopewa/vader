classdef trapezoidalStepAngleControl
    %trapezoidalStepAngleControl provides the linear and angular velocity
    % required to turn in-place by a given angle by accelerating as fast as
    % possible to a maximum angular velocity
    
    properties
        amax
        vmax
        theta
        sgn
        tPause
        tFinal
    end
    
    methods
        function obj = trapezoidalStepAngleControl(amax, vmax, theta, sign, tPause)
            obj.amax = amax;
            obj.vmax = vmax;
            obj.theta = theta;
            obj.sgn = sign;
            obj.tPause = tPause;
            obj.tFinal = (theta + vmax^2/amax)/vmax;
        end
    
        function [V, w] = computeControl(obj, timeNow)
            % Return the linear and angular velocity that the robot should
            % be executing at time tnow. Any pauses specified in the
            % contructor are implemented here too.
            V = 0;
            if (timeNow < obj.tPause || timeNow > obj.tFinal + obj.tPause)
                w = 0;
            else
                timeNow = timeNow - obj.tPause;
                t_ramp = obj.vmax/obj.amax;
                
                if 0 < timeNow && timeNow < t_ramp
                    w = obj.sgn * obj.amax*timeNow;
                elseif 0 < obj.tFinal-timeNow && obj.tFinal-timeNow < t_ramp
                    w = obj.sgn * obj.amax * (obj.tFinal - timeNow);
                elseif t_ramp < timeNow && timeNow < obj.tFinal-t_ramp
                    w = obj.sgn * obj.vmax;
                else
                    w = 0;
                end
            end
        end
    
        function duration = getTrajectoryDuration(obj)
            % Return the total time required for motion and initial and
            % terminal pauses.
            duration = obj.tFinal + (2*obj.tPause);
        end
    end
end

