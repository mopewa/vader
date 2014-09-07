function [ x, y, theta] = irToXy( i, r )
% irToXy finds position and bearing of a range pixel endpoint
% Finds the position and bearing of the endpoint of a range pixel in the plane.

start_angle = 0;
angle_spacing = 1;

theta = start_angle + (i-1) * angle_spacing;

x = r * cos(theta * (pi/180));

y = r * sin(theta * (pi/180));

% theta needs to be from -180 to 180

if (theta > 180) 
    theta = theta - 360;
end

end