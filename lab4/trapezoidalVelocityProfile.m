function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sgn)
    t_ramp = vmax/amax;
    t_f = (dist + vmax^2/amax)/vmax;
    if 0 < t && t < t_ramp
        uref = sgn * amax*t;
    elseif 0 < t_f-t && t_f-t < t_ramp
        uref = sgn * amax * (t_f - t);
    elseif t_ramp < t && t < t_f-t_ramp
        uref = sgn * vmax;
    else
        uref = 0;
    end
end
