function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
%modelDiffSteerRobot models the robot's position given left and right wheel
% velocity vectors
    robotWidth = 237; % in mm
    
    numSteps = (tf-t0)/dt + 1;
    x = zeros(ceil(numSteps), 1);
    y = zeros(ceil(numSteps), 1);
    th = zeros(ceil(numSteps), 1);
    currentIter = 1;
    currentX = 0;
    currentY = 0;
    theta = 0;
    
    myPlot = plot(x, y, 'b-');
    xlim([-0.5 0.5]);
    ylim([-0.5 0.5]);
    pause(1);
    
    while (currentIter < numSteps)
        pause(.0001);
        currentIter = currentIter + 1;
        
        V = (vr(currentIter)+vl(currentIter))/2000; % in m/s
        omega = (vr(currentIter)-vl(currentIter))/robotWidth;
        
        theta = theta + omega*dt/2;
        currentX = currentX + V*cos(theta)*dt;
        currentY = currentY + V*sin(theta)*dt;
        
        set(myPlot, 'xdata', [get(myPlot,'xdata') currentX], ...
            'ydata', [get(myPlot,'ydata') currentY]);
        
        x(currentIter) = currentX;
        y(currentIter) = currentY;
        th(currentIter) = theta;
        
        theta = theta + omega*dt/2;
    end
end

