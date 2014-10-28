classdef robotKeypressDriver
    %robotKeypressDriver Creates a keyboard event handler and then lets the
    %user drive the robot with the arrow keys.

    properties (Constant)
    
    end
    
    properties (Access = private)
        fh=[];
        robot;
    end
   
    properties (Access = public)

    end    
    
    methods (Static = true)

        function drive(robot,vGain)
            % drive the robot
            Vmax = 0.02*vGain;
            dV = 0.002*vGain;
            key = pollKeyboard();
            
            if (key ~= false)
                if (strcmp(key,'uparrow'))
                    disp('up');
                    robot.sendVelocity(Vmax,Vmax);
                elseif (strcmp(key,'downarrow'))
                    disp('down');
                    robot.sendVelocity(-Vmax,-Vmax);
                elseif (strcmp(key,'leftarrow'))
                    disp('left');
                    robot.sendVelocity(-Vmax,Vmax);
                elseif(strcmp(key,'rightarrow'))
                    disp('right');
                    robot.sendVelocity(Vmax,-Vmax);
                elseif(strcmp(key,'s'))
                    disp('stop');
                    robot.sendVelocity(0.0,0.0);
                end;
            end;
        end
    end
    
    methods (Access = private)

    end
    
    methods (Access = public)

        function obj = robotKeypressDriver(robot, fh)
            % create a robotKeypressDriver for the figure handle
            % normally you call this with gcf for fh
            obj.fh = fh;
            obj.robot = robot;
            set(fh,'KeyPressFcn',@(source, event) keyboardEventListener(source, event, robot));
        end
    end
end

function keyboardEventListener(~,event, robot)
    %keyboardEventListener Invoked when a keyboard character is pressed.

    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    keypressKey = event.Key;
    
    robotKeypressDriver.drive(robot, 1);

end

function res = pollKeyboard()
    %pollKeyboard Waits until the callback says there is new data.
    % This routine is useful when you want to be able to capture
    % and respond to a keypress as soon as it arrives.

    % To use this, execute the following line:
    % kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
    % before calling this function.

    global keypressDataReady;
    global keypressKey;
    keyboardDataReadyLast = keypressDataReady;
    keypressDataReady = 0;
    
    if (keyboardDataReadyLast)
        res = keypressKey;
        disp('gotOne');
    else
        res = false;
    end
end    
    


