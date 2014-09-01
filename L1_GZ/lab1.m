%% Lab 1 Task 1 Move The Robot

robot = neato('sim');
v = 0.05;
tic;
tstart = tic;

while(toc(tstart) < 4) 
    robot.sendVelocity(v, v)
    pause(0.005);
end

tstart = tic;
while(toc(tstart) < 4) 
    robot.sendVelocity(-v, -v)
    pause(0.005);
end

robot.sendVelocity(0, 0);
robot.shutdown();

%% Lab 1 Task 2 Move By Distance

robot = neato('sim');
v = 0.05;
encoder = 6543;
initEncoder = encoder
tic;

while (encoder - initEncoder < 200)
    tstart = tic;
    robot.sendVelocity(v, v);
    pause(0.001);
    t = toc(tstart);
    encoder = encoder + (t*v)*1000;
end

robot.sendVelocity(0, 0);
robot.shutdown();

%% Lab 1 Task 3 Basic Plotting

timeArray = zeros(1,1);
distArray = zeros(1,1);

i = 2;
timeArray(1) = 0;
distArray(1) = 0;
time = 0.0;

robot = neato('sim');
v = 0.05;
encoder = 6543;
initEncoder = encoder
tic;

while (encoder - initEncoder < 200)
    tstart = tic;
    robot.sendVelocity(v, v);
    pause(0.001);
    t = toc(tstart);
    encoder = encoder + (t*v)*1000;
    time = time + t;
    timeArray(i) = time;
    distArray(i) = (encoder - initEncoder)/10;
    i = i + 1;
end

robot.sendVelocity(0, 0);
robot.shutdown();

plot(timeArray, distArray);

%% Lab 1 Task 4 Its Getting Real



timeArray = zeros(1,1);
distArray = zeros(1,1);

i = 2;
timeArray(1) = 0;
distArray(1) = 0;
time = 0.0;

robot = neato('nano')

v = 0.05;
encoder = robot.encoders.data.left
initEncoder = encoder
tic;

while (encoder - initEncoder < 200)
    tstart = tic;
    robot.sendVelocity(v, v);
    pause(0.001);
    t = toc(tstart);
    encoder = robot.encoders.data.left
    time = time + t;
    timeArray(i) = time;
    distArray(i) = (encoder - initEncoder)/10;
    i = i + 1;
end

while (encoder - initEncoder > 0)
    tstart = tic;
    robot.sendVelocity(-v, -v);
    pause(0.001);
    t = toc(tstart);
    encoder = robot.encoders.data.left
    time = time + t;
    timeArray(i) = time;
    distArray(i) = (encoder - initEncoder)/10;
    i = i + 1;
end

robot.sendVelocity(0, 0);
robot.shutdown();

plot(timeArray, distArray);
