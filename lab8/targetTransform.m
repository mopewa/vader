function [ x, y, th ] = targetTransform( x0, y0 ,th0 )
%TARGETTRANSFORM Summary of this function goes here
%   Detailed explanation goes here

    % describes relative pose between object and final robot position
    Tog = [1, 0, -0.178; 0, 1, 0; 0, 0, 1];
    % describes pose of object relative to sensor
    Tso = [cos(th0), -sin(th0), x0; sin(th0), cos(th0), y0; 0, 0, 1];
    % describes pose of sensor relative to robot
    Trs = [1, 0, -0.11; 0, 1, 0; 0, 0, 1];
    
    Tf = Trs * Tso * Tog;
    
    x = Tf(1,3);
    y = Tf(2,3);
    th = atan2(Tf(2,1), Tf(1,1));

end

