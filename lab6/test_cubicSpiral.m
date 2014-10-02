curve = cubicSpiral([-3.0 1.55 3.0],10001);
pose = curve.getFinalPose();
% plot(curve.poseArray(1,:), curve.poseArray(2,:));
fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));
goalX = pose(1);
goalY = pose(2);
goalTheta = pose(3);

curve2 = cubicSpiral([-3.0 1.55 3.0],125);
pose = curve2.getFinalPose();
% plot(curve.poseArray(1,:), curve.poseArray(2,:));
fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));
realX = pose(1);
realY = pose(2);
realTheta = pose(3);

plot(curve.poseArray(1,:), curve.poseArray(2,:), ...
    curve2.poseArray(1,:), curve2.poseArray(2,:));

totalErrorInMm = sqrt((goalX-realX)^2 + (goalY-realY)^2)*1000