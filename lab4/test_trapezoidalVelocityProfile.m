times = 0:.01:5;
velocs = [];
delayedVelocs = [];

dist = 1;
amax = .25 * 3;
vmax = .25;
sgn = 1;

for t = times
    velocs = [velocs trapezoidalVelocityProfile(t, amax, vmax, dist, sgn)];
    delayedVelocs = [delayedVelocs ...
        delayedTrapezoidalVelocityProfile(t, amax, vmax, dist, sgn)];
end

plot(times, velocs, times, delayedVelocs);
    