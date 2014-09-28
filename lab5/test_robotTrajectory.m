figure8 = figure8ReferenceControl(.5, .5, 2);
trap = trapezoidalStepReferenceControl(.75, .25, 1, 1, 2); 

traj = robotTrajectory(trap, [0,0,0], 0);

times = (1:800)/800*trap.getTrajectoryDuration();
poses = traj.getPoseAtTime(times);
distances = traj.getDistanceAtTime(times);
velocities = traj.getVelocityAtTime(times);

figure(1);
hold on;
last = robotTrajectory.numSamples+1;
% plot(poses(:,1), poses(:,2));
% disp([poses(800,1) poses(800,2)]);
% disp([traj.poses(last, 1) traj.poses(last, 2)]);
plot(times, distances);