classdef vaderBot
    %VADERBOT Class used for storing the state of the robot
    
    properties
        xPos         % Array of X positions of the robot
        yPos         % Array of Y positions of the robot
        theta        % Array of Theta bearings of the robot
        time         % Array of timestamps of the robot
        index        % Array index of the current state of the robot
        robot        % Neato robot object
        posPlot      % Plot of the position estimate of the robot
        localizer    % Instance of lineMapLocalizer
        xTrajectories% Set of all trajectory xPos followed
        yTrajectories% Set of all trajectory yPos followed
    end
    
    properties (Constant)
        W = 9.25*2.54/100;         % Wheel tread in m (values from Prof. Kelly's code)
        W2 = 9.25*2.54/2/100;      % 1/2 wheel tread in m
        
        maxWheelVelocity = 0.3 % max of either wheel in m/sec
        
        rad = .165;             % robot body radius id 12.75/2 inches
        frontOffset = 6*0.0254; % front surface is 6 in fwd of axle center
        objOffset = 1.5*0.0254; % half of object width
        laser_l = -0.100;       % laser offset
        laser_rad = 0.04;       % laser housing radius
        
        tdelay = 0.25;          % comms delay (bidirectional)
    end
    
    methods (Static)
        function [V, w] = vlvrToVw(vl, vr)
            % Converts wheel speeds to body linear and angular velocity.
            V = (vr + vl)/2;
            w = (vr - vl)/vaderBot.W;
        end
        
        function [vl, vr] = VwTovlvr(V, w)
            % Converts body linear and angular velocity to wheel speeds.
            vr = V + vaderBot.W2 * w;
            vl = V - vaderBot.W2 * w;
        end
        
        function bodyPts = bodyGraph()
            % return an array of points that can be used to plot therobot
            % body in a window.
            
            % angle arrays
            step = pi/20;
            q1 = 0:step:pi/2;
            q2 = pi/2:step:pi;
            cir = 0:step:2*pi;
            
            % circle for laser
            lx = vaderBot.laser_rad*-cos(cir) + vaderBot.laser_l;
            ly = vaderBot.laser_rad*sin(cir);
            
            % body rear
            bx = [-sin(q1)*vaderBot.rad lx [-sin(q2) 1 1 0]*vaderBot.rad];
            by = [-cos(q1)*vaderBot.rad ly [-cos(q2) 1 -1 -1]*vaderBot.rad];
            
            %create homogeneous points
            bodyPts = [bx ; by ; ones(1,size(bx,2))];
            
        end
        
        function senToWorld = senToWorld(robPose)
            % Finds the sensor pose in world given the robot
            % pose in the world.
            senToRob = pose(vaderBot.laser_l,0,0);
            senToWorld = robPose.bToA()*senToRob.bToA();
        end
        
        function robToWorld = robToWorld(senPose)
            % Finds the robot pose given the sensor pose in
            % the world.
            senToRob = pose(vaderBot.laser_l,0,0);
            robToWorld = senPose.bToA()*senToRob.aToB();
        end
    end
    
    methods
        function obj = vaderBot(x, y, th, r)
            obj.xPos(1) = x;
            obj.yPos(1) = y;
            obj.theta(1) = th;
            obj.time = 0;
            obj.index = 1;
            obj.robot = r;
            
            %{
            figure(1);
            hold on;
            obj.posPlot = plot(obj.xPos, obj.yPos,'r-');
            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
            %}
            
        end
        
        function obj = setLocalizer(obj, localizer)
            obj.localizer = localizer;
        end
        
        function drive(obj, lVeloc, rVeloc)
            if lVeloc > .3 || rVeloc > .3
                disp('adjusting velocity too high');
                lVeloc = min(lVeloc, .3);
                rVeloc = min(rVeloc, .3);
            elseif lVeloc < -0.3 || rVeloc < -0.3
                disp('adjusting velocity too low');
                lVeloc = max(lVeloc, -0.3);
                rVeloc = max(rVeloc, -0.3);
            end
            
            if lVeloc > .3 || rVeloc > .3
                disp('adjusting velocity too high');
                lVeloc = min(lVeloc, .3);
                rVeloc = min(rVeloc, .3);
            elseif lVeloc < -0.3 || rVeloc < -0.3
                disp('adjusting velocity too low');
                lVeloc = max(lVeloc, -0.3);
                rVeloc = max(rVeloc, -0.3);
            end
            
            obj.robot.sendVelocity(lVeloc, rVeloc);
        end
        
        function obj = setPose(obj, pose)
            if(abs(pose.x-obj.getPose.x) < 0.20 || abs(pose.y-obj.getPose.y) < 0.2)
                obj.xPos(obj.index+1) = pose.x;
                obj.yPos(obj.index+1) = pose.y;
                obj.theta(obj.index+1) = pose.th;
                obj.time(obj.index+1) = obj.time(obj.index);
                obj.index = obj.index + 1;
            else
                disp('Localization not updated');
            end
        end
        
        function curPose = getPose(obj)
            curPose = pose(obj.xPos(obj.index), obj.yPos(obj.index), obj.theta(obj.index));
        end
        
        % updates the robot's state when new laser range data arrives
        function [obj, success] = processRangeImage(obj, image, maxIters)
            %             maxIters = 10;
            curPose = obj.getPose();
            [success, poseMap] = obj.localizer.refinePose(curPose,[image.xArray; image.yArray; ones(size(image.xArray))],maxIters);
            
            if (success)
                subPose = pose.subtractPoses(curPose, poseMap);
                fractionPose = pose(.25 * subPose.x, .25*subPose.y, .25*subPose.th);
                poseFused = pose.addPoses(curPose, fractionPose);

                if (isnan(poseFused.th) || isinf(poseFused.th))
                    poseMap.th
                    %pause(10);
                else
                    obj = obj.setPose(poseFused);
                end
            end
        end
        
        % updates the robot's state when new odometry data arrives
        function obj = processOdometryData(obj, encoderL, encoderR, dt)
            obj = obj.updateState(encoderL, encoderR, dt);
        end
        
        % deprecated, use processOdometryData
        function obj = updateState(obj, encoderL, encoderR, dt)
            if (dt ~= 0)
                obj.time(obj.index+1) = obj.time(obj.index) + dt;
                
                
                vL = encoderL/dt/1000;
                vR = encoderR/dt/1000;
                w = (vR - vL)/vaderBot.W;
                V = (vR + vL)/2;
                
                %                 x1 = V*cos(obj.theta(obj.index));
                %                 y1 = V*sin(obj.theta(obj.index));
                %
                %                 x2 = V*cos(obj.theta(obj.index)+w*dt/2);
                %                 y2 = V*sin(obj.theta(obj.index)+w*dt/2);
                %
                %                 x4 = V*cos(obj.theta(obj.index)+w*dt);
                %                 y4 = V*sin(obj.theta(obj.index)+w*dt);
                %
                %                 obj.xPos(obj.index+1) = obj.xPos(obj.index)+dt*(x1+4*x2+x4)/6;
                %                 obj.yPos(obj.index+1) = obj.yPos(obj.index)+dt*(y1+4*y2+y4)/6;
                %                 obj.theta(obj.index+1) = obj.theta(obj.index)+w*dt;
                
                
                tempTheta = obj.theta(obj.index) + w*dt/2;
                
                deltaX = abs(obj.xPos(obj.index) - (obj.xPos(obj.index) + V*cos(tempTheta)*dt));
                deltaY = abs(obj.yPos(obj.index) - (obj.yPos(obj.index) + V*sin(tempTheta)*dt));
                
                if (deltaX > .2 || deltaY > .2)
                    obj.xPos(obj.index+1) = obj.xPos(obj.index);
                    obj.yPos(obj.index+1) = obj.yPos(obj.index);
                    disp('Odometry state update ignored')
                else
                    obj.xPos(obj.index+1) = obj.xPos(obj.index) + V*cos(tempTheta)*dt;
                    obj.yPos(obj.index+1) = obj.yPos(obj.index) + V*sin(tempTheta)*dt;                 
                end
                obj.theta(obj.index+1) = tempTheta + w*dt/2;
                
                
                obj.index = obj.index+1;
                %{
                set(obj.posPlot,'xdata', [get(obj.posPlot,'xdata') ...
                    obj.xPos(obj.index)],'ydata', [get(obj.posPlot,'ydata') ...
                    obj.yPos(obj.index)]);
                plot(obj.posPlot);
                %}
            end
        end
        
        % turn in place by a given angle
        function [obj, totalError] = moveRelAngle(obj, theta)
            sign = 1;
            if (theta < 0)
                theta = abs(theta);
                sign = -1;
            end
            vmax = min(theta, 1);
            path = trapezoidalStepAngleControl(.75, vmax, theta, sign, 0);
            [obj, totalError] = obj.executeTrajectory2(path, 1, 1, 0);
        end
        
        % move a fixed straight line distance relative to the robot
        function [obj, totalError] = moveRelDistance(obj, dist)
            sign = 1;
            if dist < 0
                dist = abs(dist);
                sign = -1;
            end
            % pass dist as vmax so that trajectory takes at least a second
            vmax = min(dist, .25);
            path = trapezoidalStepReferenceControl(.75, vmax, dist, sign, 0);
            [obj, totalError] = obj.executeTrajectory2(path, 1, 0, 0);
        end
        
        % execute a trajectory to a pose specified in world coordinates
        function [obj, totalError] = executeTrajectoryToAbsolutePose(obj, absPose, useMap)
            relPose = pose(obj.getPose().aToB()*absPose.bToA());
            [obj, totalError] = obj.executeTrajectoryToRelativePose(relPose, useMap);
        end
        
        % execute a trajectory to a pose specified in robot coordinates
        function [obj, totalError] = executeTrajectoryToRelativePose(obj, pose, useMap)
            path = cubicSpiral.planTrajectory(pose.x, pose.y, pose.th, 1);
            path.planVelocities(.15);
            [obj, totalError] = obj.executeTrajectory(path, useMap);
        end
        
        % for backwards-compatability, for non turn-in-place trajectories
        function [obj, totalError] = executeTrajectory(obj, trajectory, useMap)
            [obj, totalError] = obj.executeTrajectory2(trajectory, useMap, 0, 1);
        end
        
        function [obj, totalError] = executeTrajectory2(obj, trajectory, useMap, turnInPlace, usePause)
            global leftEncoder;
            global rightEncoder;
            global timeStamp;
            
            startPose = [obj.xPos(obj.index),obj.yPos(obj.index),obj.theta(obj.index)];
            
            traj = robotTrajectory(trajectory, startPose, 0);
            
            follower = trajectoryFollower(traj, startPose);
            % use theta-only controller for turn-in-place trajectories
            follower = follower.setTurnInPlace(turnInPlace);
            
            prevLeftEncoder = leftEncoder;
            prevRightEncoder = rightEncoder;
            prevTimeStamp = timeStamp;
            
            if (usePause)
                pause(2);
            end
            timer = tic;
            currentTime = toc(timer);
            while (currentTime < trajectory.getTrajectoryDuration() + 1)
%                 obj.robot.encoders.data.left
                newLE = leftEncoder;
                newRE = rightEncoder;
                newTS = timeStamp;
                eL = newLE - prevLeftEncoder;
                eR = newRE - prevRightEncoder;
                dt = newTS - prevTimeStamp;
                
                prevLeftEncoder = newLE;
                prevRightEncoder = newRE;
                prevTimeStamp = newTS;
                
                obj = obj.processOdometryData(eL, eR, dt);
                
                currentTime = toc(timer);
                [vl, vr, follower] = follower.getVelocity(currentTime, obj);
                
                obj.drive(vl, vr);
                % process range data
                if (useMap && currentTime < trajectory.getTrajectoryDuration() - 0.2)
                    ranges = obj.robot.laser.data.ranges;
                    downSample = 20;
                    image = rangeImage(ranges, downSample, false);
                    [obj, success] = obj.processRangeImage(image, 10);
                end
                pause(.005);
            end
            
            eL = leftEncoder - prevLeftEncoder;
            eR = rightEncoder - prevRightEncoder;
            dt = timeStamp - prevTimeStamp;
            
            prevLeftEncoder = leftEncoder;
            prevRightEncoder = rightEncoder;
            prevTimeStamp = timeStamp;
            
            obj = obj.updateState(eL, eR, dt);
            
            obj.drive(0,0);
            
            eL = leftEncoder - prevLeftEncoder;
            eR = rightEncoder - prevRightEncoder;
            dt = timeStamp - prevTimeStamp;
            
            obj = obj.updateState(eL, eR, dt);
            
            [finalX, finalY, ~] = traj.getPoseAtTime(currentTime);
            xError = obj.xPos(obj.index)-finalX;
            yError = obj.yPos(obj.index)-finalY;
            totalError = sqrt(xError^2 + yError^2);
            
            %             figure(5);
            %             hold on;
            %             plot(obj.xPos, obj.yPos, 'b');
            %             obj.xTrajectories = [obj.xTrajectories, traj.poses(:,1)'];
            %             obj.yTrajectories = [obj.yTrajectories, traj.poses(:,2)'];
            %             plot(obj.xTrajectories, obj.yTrajectories, 'g');
            %             figure(6);
            %             plot(follower.time, follower.error, '-r');
        end
        
        
        function [obj, success] = pickDropObject(obj, robot, p, dropFlag, nextPose)
            % takes robot and targetTransformed object pose. If dropFlag is false,
            % it drives to pose, does a pick up and backs up. Otherwise, it
            % drive to pose and does a drop
            
            success = 1;
            
            if(~dropFlag)
                % go to pose
                [obj, ~] = obj.executeTrajectoryToRelativePose(p, 1);
                
                pause(1);
            
                %Find target
                ranges = robot.laser.data.ranges;
                image = rangeImage(ranges, 1, 1);
                [x,y,th] = image.findObjectAround(.14, 0);
                foundLine = x || y;
                if (foundLine)
                    pause(.1);
                    targetPose = pose(x, y, th);
                    [pX, pY, pTh] = targetTransform(targetPose.x, targetPose.y, targetPose.th);
                    p = pose(pX, pY, pTh);
                else
                    success = 0;
                end
            end
            
            % go to pose
            if (success)
                [obj, ~] = obj.executeTrajectoryToRelativePose(p, 1);
   
                pause(1);

                if (dropFlag)
                    robot.forksDown();
                else
                    obj = obj.moveRelDistance(.08);
                    robot.forksUp();
                end

                pause(1);

                % back up 15 centimeters
                obj = obj.moveRelDistance(-.15);

                relNextTarget = pose(obj.getPose().aToB()*nextPose.bToA());
                nextTargetTheta = atan2(relNextTarget.y, relNextTarget.x);
                obj = obj.moveRelAngle(nextTargetTheta);
            end
        end
    end
    
end

