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
    end
    
    methods
        function obj = vaderBot(x, y, th, r)
            obj.xPos(1) = x;
            obj.yPos(1) = y;
            obj.theta(1) = th;
            obj.time = 0;
            obj.index = 1;
            obj.robot = r;
            
            figure(1);
            hold on;
            obj.posPlot = plot(obj.xPos, obj.yPos,'r-');
            xlim([-0.5 0.5]);
            ylim([-0.5 0.5]);
        end
        
        function drive(obj, lVeloc, rVeloc)
            obj.robot.sendVelocity(lVeloc, rVeloc);
        end
        
        function obj = updateState(obj, encoderL, encoderR, dt)
            if (dt ~= 0)
                obj.time(obj.index+1) = obj.time(obj.index) + dt;
                
                vL = encoderL/dt/1000;
                vR = encoderR/dt/1000;
                w = (vR - vL)/vaderBot.W;
                V = (vR + vL)/2;
                
                x1 = V*cos(obj.theta(obj.index));
                y1 = V*sin(obj.theta(obj.index));
                
                x2 = V*cos(obj.theta(obj.index)+w*dt/2);
                y2 = V*sin(obj.theta(obj.index)+w*dt/2);
                
                x4 = V*cos(obj.theta(obj.index)+w*dt);
                y4 = V*sin(obj.theta(obj.index)+w*dt);
                
                obj.xPos(obj.index+1) = obj.xPos(obj.index)+dt*(x1+4*x2+x4)/6;
                obj.yPos(obj.index+1) = obj.yPos(obj.index)+dt*(y1+4*y2+y4)/6;
                obj.theta(obj.index+1) = obj.theta(obj.index)+w*dt;
                
                tempTheta = obj.theta(obj.index) + w*dt/2;
                obj.xPos(obj.index+1) = obj.xPos(obj.index) + V*cos(tempTheta)*dt;
                obj.yPos(obj.index+1) = obj.yPos(obj.index) + V*sin(tempTheta)*dt;
                obj.theta(obj.index+1) = tempTheta + w*dt/2;
                
                obj.index = obj.index+1;
                
                set(obj.posPlot,'xdata', [get(obj.posPlot,'xdata') ...
                    obj.xPos(obj.index)],'ydata', [get(obj.posPlot,'ydata') ...
                    obj.yPos(obj.index)]);
                plot(obj.posPlot);
            end
        end
        
        function executeTrajectory(obj, trajectory)
            global leftEncoder;
            global rightEncoder;
            global timeStamp;
            
            traj = robotTrajectory(trajectory, [0,0,0], 0);
            follower = trajectoryFollower(traj);
            
            prevLeftEncoder = leftEncoder;
            prevRightEncoder = rightEncoder;
            prevTimeStamp = timeStamp;
            
            timer = tic;
            currentTime = toc(timer);
            
            while (currentTime < trajectory.getTrajectoryDuration() + 1)
                eL = leftEncoder - prevLeftEncoder;
                eR = rightEncoder - prevRightEncoder;
                dt = timeStamp - prevTimeStamp;
                
                prevLeftEncoder = leftEncoder;
                prevRightEncoder = rightEncoder;
                prevTimeStamp = timeStamp;
                
                obj = obj.updateState(eL, eR, dt);
                
                currentTime = toc(timer);
                [vl, vr, follower] = follower.getVelocity(currentTime, obj);
                
                obj.drive(vl, vr);
                
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
            
            %obj = obj.updateState(eL, eR, dt);
            
            [finalX, finalY, finalTheta] = traj.getPoseAtTime(currentTime);
            xError = obj.xPos(obj.index)-finalX;
            yError = obj.yPos(obj.index)-finalY;
            totalError = sqrt(xError^2 + yError^2)
            
            %figure(1);
            %plot(obj.xPos, obj.yPos, traj.poses(:,1)', traj.poses(:,2)');
            %figure(2);
            %plot(follower.time, follower.error, '-r');
        end
    end
    
end

