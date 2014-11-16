function targetPose = targetTransform2(pose0)
%TARGETTRANSFORM Given a target pose in the *ROBOT* frame, returns the pose
% the robot should drive to to reach the target

    x0 = pose0.x;
    y0 = pose0.y;
    th0 = pose0.th;

    % describes relative pose between object and final robot position
    Tog = [1, 0, -0.178; 0, 1, 0; 0, 0, 1];
    % describes pose of object relative to sensor
    Tso = [cos(th0), -sin(th0), x0; sin(th0), cos(th0), y0; 0, 0, 1];
    % moves the final pose forward or backward relative to the target
    backspace = 0; % negative to move backward
    Trs = [1, 0, backspace; 0, 1, 0; 0, 0, 1];
    
    Tf = Trs * Tso * Tog;
    
    x = Tf(1,3);
    y = Tf(2,3);
    th = atan2(Tf(2,1), Tf(1,1));

    targetPose = pose(x, y, th);
end

