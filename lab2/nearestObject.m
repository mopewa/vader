function [ min_x, min_y, min_theta ] = nearestObject( lidar_data )
% nearestObject finds the position and bearing of the nearest object. 
% Finds the position and bearing of the nearest object given lidar values,
% returns all [maxObjectRange, maxObjectRange, 0] if there are no objects 
% within the maxObjectRange or the maxBearing

maxObjectRange = 2;
% filter out noise
minRange = .1;
maxBearing = 90;

valid_indices = find(lidar_data > minRange & lidar_data < maxObjectRange);

current_min = maxObjectRange + 10;
min_x = maxObjectRange;
min_y = maxObjectRange;
min_theta = 0;

for i = 1:length(valid_indices)
    ind = valid_indices(i);
    range_value = lidar_data(ind);
    if (range_value < current_min)
        current_min = range_value;
        [x, y, theta] = irToXy(ind, range_value);
        if (abs(theta) < maxBearing)
            min_x = x;
            min_y = y;
            min_theta = theta;
        end
    end
end
end

