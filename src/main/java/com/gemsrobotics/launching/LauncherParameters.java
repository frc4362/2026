package com.gemsrobotics.launching;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public record LauncherParameters(Rotation2d hoodAngle, double rps) implements Interpolatable<LauncherParameters> {
    @Override
    public LauncherParameters interpolate(final LauncherParameters other, final double t) {
        return new LauncherParameters(
                this.hoodAngle.interpolate(other.hoodAngle, t),
                Interpolator.forDouble().interpolate(this.rps, other.rps, t));
    }
}
