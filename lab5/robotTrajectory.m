classdef robotTrajectory
    %robotTrajectory given a reference control, samples time, distances,
    % velocities, and poses, and uses interpolation to provide access to
    % any of these properties at arbitrary times.
    
    properties
        times           
        distances
        velocities
        poses
    end
    
    properties (Constant)
        % Chosen somewhat arbitrarily so that errors are pretty small.
        numSamples = 1000;
    end
    
    methods
        function obj = robotTrajectory(referenceControl, initPose, initDistance)
            % Generates numSamples samples for time, pose, distance, and
            % velocity for the path specified by given reference control.
            % initPose is of the form [initX, initY, initTheta].
            finalTime = referenceControl.getTrajectoryDuration();
            dt = finalTime / obj.numSamples;
            obj.times = (0:obj.numSamples)*dt;
            obj.distances = zeros(1, obj.numSamples+1);
            obj.velocities = zeros(obj.numSamples+1,2);
            obj.poses = zeros(obj.numSamples+1,3);
            obj.poses(1,:) = initPose;
            obj.distances(1) = initDistance;
            for i = 1:obj.numSamples
                time = obj.times(i);
                [V, w] = referenceControl.computeControl(time);
                obj.velocities(i,:) = [V, w];
                prevPose = obj.poses(i,:);
                prevX = prevPose(1);
                prevY = prevPose(2);
                prevTheta = prevPose(3);
                theta = prevTheta + w * dt/2;
                curX = prevX + V*cos(theta)*dt;
                curY = prevY + V*sin(theta)*dt;
                theta = theta + w * dt/2;
                obj.poses(i+1,:) = [curX, curY, theta];
                obj.distances(i+1) = obj.distances(i) + V*dt;
            end
        end
        
        function [x, y, theta] = getPoseAtTime(obj, t)
            t = t - 0.28;
            % returns [x, y, theta] at time t
            if (t > max(obj.times))
                pose = obj.poses(obj.numSamples+1, :);
            else
                pose = interp1(obj.times, obj.poses, t);
            end
            x = pose(1);
            y = pose(2);
            theta = pose(3);
        end
        
        function [V, w] = getVelocityAtTime(obj, t)
            % returns [V, w] at time t
            if (t > max(obj.times))
                velocity = obj.velocities(obj.numSamples, :);
            else
                velocity = interp1(obj.times, obj.velocities, t);
            end
            V = velocity(1);
            w = velocity(2);
        end
        
        function distance = getDistanceAtTime(obj, t)
            % returns total distance travelled at time t
            if (t > max(obj.times))
                distance = obj.distances(obj.numSamples+1);
            else
                distance = interp1(obj.times, obj.distances, t);
            end
        end
    end
    
end

