%% robot setup 

robot = neato('centi');

pause(5);

%% Case 1: Read encoder, Matlab clock
figure(1);
hold on;

timer = tic;
timeStamp = toc(timer);
leftEncoder = robot.encoders.data.left;

robot.sendVelocity(.03,.03);
while(toc(timer) < 15)
    lastTime = timeStamp;
    lastEncoder = leftEncoder;
    timeStamp = toc(timer);
    if (leftEncoder > .1) % noise filter
        leftEncoder = robot.encoders.data.left;  
        ds = abs(lastEncoder - leftEncoder);
        dt = abs(lastTime - timeStamp);
              
        V = (ds/dt)/1000;   % in m/s
        plot(timeStamp,V, 'x')
    end
    pause(.02)
end
%% Case 2 and 3: See listener for details

figure(2);
hold on;
lh = event.listener(robot.encoders,'OnMessageReceived', ...
    @neatoEncoderEventListener);

pause(3);
robot.sendVelocity(.03,.03);
pause(2);
robot.sendVelocity(0,0);
pause(2);
delete(lh);

