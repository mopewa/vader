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
                [vl, vr] = vaderBot.VwTovlvr(V, w);
                obj.velocities(i,:) = [vl, vr];
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
        
        function pose = getPoseAtTime(obj, t)
            % returns [x, y, theta] at time t
            pose = interp1(obj.times, obj.poses, t);
        end
        
        function velocity = getVelocityAtTime(obj, t)
            % returns [vl, vr] at time t
            velocity = interp1(obj.times, obj.velocities, t);
        end
        
        function distance = getDistanceAtTime(obj, t)
            % returns total distance travelled at time t
            distance = interp1(obj.times, obj.distances, t);
        end
    end
    
end

