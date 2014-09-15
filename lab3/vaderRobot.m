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
            xPos(1) = x;
            yPos(1) = y;
            theta(1) = th;
            time = 0;
            index = 1;
            robot = r;
            L = 0.235;
            
            posPlot = plot(xPos, yPos,'r-');
            xlim([0.0 0.5]);
            ylim([0.0 0.5]);
        end
        function drive(lVeloc, rVeloc)
           sendVelocity(robot, lVeloc, rVeloc); 
        end
        function updateState(encoderL, encoderR, dt)
            time(index+1) = time(index) + dt;
            
            vL = encoderL/dt/1000;
            vR = encoderR/dt/1000;
            w = (vR - vL)/L;
            V = (vR + vL)/2;
            
            tempTheta = theta(index) + w*dt/2;
            xPos(index+1) = xPos(index) + V*cos(tempTheta)*dt;
            yPos(index+1) = yPos(index) + V*sin(tempTheta)*dt;
            theta(index+1) = tempTheta + w*dt/2;
            
            index = index+1;
            
            set(posPlot,'xdata', [get(posPlot,'xdata') xPos(index)],'ydata', [get(posPlot,'ydata') yPos(index)]);
        end
    end
    
end

