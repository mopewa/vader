classdef vaderRobot
    %VADERROBOT Class used for storing the state of the robot
    
    properties
        xPos         % Array of X positions of the robot
        yPos         % Array of Y positions of the robot
        theta        % Array of Theta bearings of the robot
        time         % Array of timestamps of the robot
        index        % Array index of the current state of the robot
        robot        % Neato robot object
        L            % Distance between wheels on the robot in meters
        posPlot      % Plot of the position estimate of the robot
    end
    
    methods
        function obj=vaderRobot(x, y, th, r)
            obj.xPos(1) = x;
            obj.yPos(1) = y;
            obj.theta(1) = th;
            obj.time = 0;
            obj.index = 1;
            obj.robot = r;
            obj.L = 0.235;
            
            obj.posPlot = plot(obj.xPos, obj.yPos,'r-');
            xlim([0.0 0.5]);
            ylim([0.0 0.5]);
        end
        function drive(obj, lVeloc, rVeloc)
           obj.robot.sendVelocity(lVeloc, rVeloc); 
        end
        function updateState(obj, encoderL, encoderR, dt)
            obj.time(obj.index+1) = obj.time(obj.index) + dt;
            
            vL = encoderL/dt/1000;
            vR = encoderR/dt/1000;
            w = (vR - vL)/obj.L;
            V = (vR + vL)/2;
            
            tempTheta = obj.theta(obj.index) + w*dt/2;
            obj.xPos(obj.index+1) = obj.xPos(obj.index) + V*cos(tempTheta)*dt;
            obj.yPos(obj.index+1) = obj.yPos(obj.index) + V*sin(tempTheta)*dt;
            obj.theta(obj.index+1) = tempTheta + w*dt/2;
            
            obj.index = obj.index+1;
            
            set(obj.posPlot,'xdata', [get(obj.posPlot,'xdata') obj.xPos(obj.index)],'ydata', [get(obj.posPlot,'ydata') obj.yPos(obj.index)]);
        end
    end
    
end

