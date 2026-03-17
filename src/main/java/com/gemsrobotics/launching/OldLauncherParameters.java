package com.gemsrobotics.launching;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public record OldLauncherParameters(Rotation2d hoodAngle, double rps) implements Interpolatable<OldLauncherParameters>, HoodAndRps {
    @Override
    public OldLauncherParameters interpolate(final OldLauncherParameters other, final double t) {
        return new OldLauncherParameters(
                this.hoodAngle.interpolate(other.hoodAngle, t),
                Interpolator.forDouble().interpolate(this.rps, other.rps, t));
    }

    @Override
    public Rotation2d getHoodAngle() {
        return this.hoodAngle;
    }

    @Override
    public double getRps() {
        return this.rps;
    }
}
