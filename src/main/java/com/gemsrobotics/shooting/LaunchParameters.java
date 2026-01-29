package com.gemsrobotics.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public record LaunchParameters(Rotation2d hoodAngle, double rps) implements Interpolatable<LaunchParameters> {
    @Override
    public LaunchParameters interpolate(final LaunchParameters other, final double t) {
        return new LaunchParameters(
                this.hoodAngle.interpolate(other.hoodAngle, t),
                Interpolator.forDouble().interpolate(this.rps, other.rps, t));
    }
}
