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
        W = 0.235;         % Distance between wheels on the robot in meters
        W2 = 0.235/2;      % W/2
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
            
%             figure(1);
%             hold on;
%             obj.posPlot = plot(obj.xPos, obj.yPos,'r-');
%             xlim([-0.5 0.5]);
%             ylim([-0.5 0.5]);
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
            
                tempTheta = obj.theta(obj.index) + w*dt/2;
                obj.xPos(obj.index+1) = obj.xPos(obj.index) + V*cos(tempTheta)*dt;
                obj.yPos(obj.index+1) = obj.yPos(obj.index) + V*sin(tempTheta)*dt;
                obj.theta(obj.index+1) = tempTheta + w*dt/2;
            
                obj.index = obj.index+1;
                
                if (obj.xPos(obj.index) >= 0.5 || obj.xPos(obj.index) <= -0.5 || obj.yPos(obj.index) >= 0.5 || obj.yPos(obj.index) <= -0.5)
                    V
                    w
                    dt
                    vL
                    vR
                end
                    
            
%                 set(obj.posPlot,'xdata', [get(obj.posPlot,'xdata') ...
%                     obj.xPos(obj.index)],'ydata', [get(obj.posPlot,'ydata') ...
%                     obj.yPos(obj.index)]);
%                 plot(obj.posPlot);
            end
        end
    end
    
end

