%% robot setup

%Must Initialize Globals
global leftEncoder;
global rightEncoder;
global timeStamp;

robot = neato('centi');

lh = event.listener(robot.encoders,'OnMessageReceived', ...
    @basicEncoderListener);

pause(10);

figure(1);
hold on;
%% Part 1.1

r = vaderBot(0, 0, 0, robot);

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

targetX = 0.3;
targetY = 0;

kp = 4;
kd = 0;
ki = 0;

integralError = 0;
error = targetX - r.xPos(r.index);
integralError = integralError + error;

timer = tic;
currentTime = toc(timer);

errorPlot = plot(currentTime, error,'r-');
xlim([0 4]);
ylim([-0.2 0.4]);

while(abs(r.xPos(r.index)-targetX) >= 0.0001)
    
    eL = leftEncoder - prevLeftEncoder;
    eR = rightEncoder - prevRightEncoder;
    dt = timeStamp - prevTimeStamp;
    
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevTimeStamp = timeStamp;

    r = r.updateState(eL, eR, dt);
    
    prevError = error;
    error = (targetX - r.xPos(r.index));
    integralError = integralError + error;
    currentTime = toc(timer);
   
    V = kp*error + kd*(error-prevError) + ki*integralError;
    
    if (V > 0.3)
        V = 0.3;
    end
    if (V < -0.3)
        V = -0.3;
    end
    
    r.drive(V, V);
    
    set(errorPlot,'xdata', [get(errorPlot,'xdata') ...
                    currentTime],'ydata', [get(errorPlot,'ydata') ...
                    error]);
    plot(errorPlot);
    
    pause(0.01);
    
end

sendVelocity(robot, 0, 0);
delete(lh)

eL = leftEncoder - prevLeftEncoder
eR = rightEncoder - prevRightEncoder
dt = timeStamp - prevTimeStamp
    
prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp

r = r.updateState(eL, eR, dt);