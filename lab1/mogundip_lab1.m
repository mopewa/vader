%% Lab 1 Task 1:Move the Robot %%
clear all
clc
robot = neato('sim');

timer = tic;

while (toc(timer) < 4.00)
    robot.sendVelocity(.05,.05);
    pause(.005);
end
robot.sendVelocity(0,0);

pause(0.005);

timer = tic;
while (toc(timer) < 4.00)
    robot.sendVelocity(-.05,-.05);
    pause(.005);
end
robot.sendVelocity(0,0);

%% Lab 1 Task 2: Basic Simulation
clear all 
clc
leftStart = 6934;
leftEncoder = leftStart;
velocityMM = 50;
signedDistance = 0;
timeArray = zeros(1,1);
distArray = zeros(1,1);
iteration = 1;

time = 0;
timer = tic;

while(signedDistance < 20)
    time = toc(timer);
    timeArray(iteration) = time;
    leftEncoder = leftStart + (velocityMM * time);
    signedDistance =  (leftEncoder - leftStart) / 10;
    distArray(iteration) = signedDistance;
    iteration = iteration + 1;
    pause(.001);
end

%% Lab 1 Task 3: Basic Plotting
plot(timeArray, distArray);
xlabel('Time (s)');
ylabel('Distance (cm)');

%% Lab 1 Task 4: Challenge/Graded Task
clear all
clc
robot = neato('centi');
signedDistanceCM = 0;
pause(2);
encoderStartCM = robot.encoders.data.left;
timeArray = zeros(1,1);
encoderArray = zeros(1,1);
iteration = 1;
timer = tic;

while (signedDistanceCM <= 20)
    robot.sendVelocity(.05,.05);
    pause(.005);
    signedDistanceCM = (robot.encoders.data.left - encoderStartCM)/10
    timeArray(iteration) = toc(timer);
    encoderArray(iteration) = signedDistanceCM;
    iteration = iteration + 1;
end
robot.sendVelocity(0,0);
pause(2);
distanceSoFar = signedDistanceCM;
signedDistanceCM = 0;
encoderStartCM = robot.encoders.data.left;

while(signedDistanceCM >= -20)
    robot.sendVelocity(-.05,-.05);
    pause(.005);
    signedDistanceCM = (robot.encoders.data.left - encoderStartCM) / 10;
    encoderArray(iteration) = distanceSoFar + signedDistanceCM;
    timeArray(iteration) = toc(timer);
    
end
robot.sendVelocity(0,0);
robot.close()
figure(2);
plot(timeArray, encoderArray);
xlabel('Time (s)');
ylabel('Distance (cm)');