%% robot setup

%Must Initialize Globals
global leftEncoder;
global rightEncoder;
global timeStamp;

robot = neato('centi');

lh = event.listener(robot.encoders,'OnMessageReceived', ...
    @basicEncoderListener);

pause(4);

figure(1);
hold on;
%% Part 1.1

r = vaderBot(0, 0, 0, robot);

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

targetX = 1;
targetY = 0;

kp = 4;
kd = 1; %.025;
ki = 0;

integralError = 0;
error = .0039;
integralError = integralError + error;

integralPos = 0;
amax = .25 * 3;
vmax = .25;
dist = targetX;
V = 0;

timer = tic;
currentTime = toc(timer);

%errorPlot = plot(currentTime, error)

errors = [error];

times = [currentTime];
xlim([0 5]);
ylim([-0.2 1.2]);

uref = 0;
u = 0;
veloc = 0;
velocs = [0];
integralPositions = [0];

encoderStart = leftEncoder;
encoders = [0];


sgn = 1;
feedforwardOnly = false;

%while(abs(r.xPos(r.index)-targetX) >= 0.0001 || V > .01)
while ((feedforwardOnly == false && abs(abs(leftEncoder - encoderStart) - (targetX*1000)) >= .0004) ...
        || (feedforwardOnly == true && currentTime < (dist + vmax^2/amax)/vmax + 1))
 %while (currentTime < (dist + vmax^2/amax)/vmax + 1)
    eL = leftEncoder - prevLeftEncoder;
    eR = rightEncoder - prevRightEncoder;
    dt = timeStamp - prevTimeStamp;
    
    prevLeftEncoder = leftEncoder;
    prevRightEncoder = rightEncoder;
    prevTimeStamp = timeStamp;
    
    r = r.updateState(eL, eR, dt);
    
    uref = trapezoidalVelocityProfile(currentTime, amax, vmax, dist, sgn);
    udelay = delayedTrapezoidalVelocityProfile(currentTime, amax, vmax, dist, sgn);
    integralPos = integralPos + udelay * dt;
    velocs = [velocs uref];
    integralPositions = [integralPositions integralPos];
    encoders = [encoders (leftEncoder-encoderStart)/1000];
    
    % r.drive(uref, uref);
    
    prevError = error;
    error = (integralPos - r.xPos(r.index));
    integralError = integralError + error * dt;
    
    errors = [errors error];
    
    currentTime = toc(timer);
    times = [times currentTime];
    
    V = kp*error + kd*(error-prevError) + ki*integralError;
    
    if (feedforwardOnly == true)
        u = uref;
    else
        u = V + uref;
    end
    
    if (u > 0.3)
        u = 0.3;
    end
    if (u < -0.3)
        u = -0.3;
    end
    
    r.drive(u, u);
    
    % plot error for feedback
    %         set(errorPlot,'xdata', [get(errorPlot,'xdata') ...
    %                         currentTime],'ydata', [get(errorPlot,'ydata') ...
    %                         error]);
    % plot(errorPlot);
    
    % plot velocity vs position for the simulator
    %     plot(times, velocs, times, integralPositions);
    
    % plot delayed reference position vs. encoder position
    plot(times, integralPositions, times, encoders);
    
    pause(0.01);
    
end

sendVelocity(robot, 0, 0);
delete(lh)

eL = leftEncoder - prevLeftEncoder;
eR = rightEncoder - prevRightEncoder;
dt = timeStamp - prevTimeStamp;

prevLeftEncoder = leftEncoder;
prevRightEncoder = rightEncoder;
prevTimeStamp = timeStamp;

r = r.updateState(eL, eR, dt);

r.xPos(r.index)

figure(2);
plot(times, errors);
