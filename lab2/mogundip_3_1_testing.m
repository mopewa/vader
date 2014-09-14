%% Robot prep
robot = neato('centi');
pause(3);
robot.startLaser();
pause(5);
%% Test irToXy
figure(1);
clf;
hold on;
line([0,0], [-3 3]);
line([-3 3], [0 0]);
% rotate camera to line up with robot frame
view([-90 90]);

% infinite running loop -- ctrl + c to end

while (true) 
    ranges = robot.laser.data.ranges;
    valid_indices = find(ranges > .1 & ranges < 1);
    for i = 1:length(valid_indices)
        ind = valid_indices(i);
        [x, y, theta] = irToXy(ind, ranges(ind));
        h = plot(x,y, 'x');
    end
    pause(1);
end

%% Test nearestObject
figure(1);
clf;
hold on;
line([0,0], [-3 3]);
line([-3 3], [0 0]);
% rotate camera to line up with robot frame (or just plot(-y,x))
view([-90 90]);

% infinite running loop -- ctrl + c to end
while (true) 
    ranges = robot.laser.data.ranges;
    [x y theta] = nearestObject(ranges);
    % rotate camera to line up with robot frame
    plot(x,y,'x');
    pause(1)
end
