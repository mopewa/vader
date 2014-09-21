function udelay = delayedTrapezoidalVelocityProfile(t, amax, vmax, dist, sgn)
    tdelay = .28;
    udelay = trapezoidalVelocityProfile(t-tdelay, amax, vmax, dist, sgn);
end

