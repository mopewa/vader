clc;

robot = neato('milli');

lh = event.listener(robot.encoders, 'OnMessageReceived', ...
    @basicEncoderListener);

r = vaderBot(.5, .5, pi()/2.0, robot);
% path = trapezoidalStepAngleControl(.75, 1, pi, 1, 1);
% traj = robotTrajectory(path, [.5,.5,pi/2], 0);
% follower = trajectoryFollower(traj, [.5,.5,pi/2]);
% follower = follower.setTurnInPlace(1);
r = r.moveRelAngle(pi);

r.getPose().x
r.getPose().y
r.getPose().th
errorPose = pose.subtractPoses(pose(.5,.5,-pi/2), r.getPose());
error = errorPose.th