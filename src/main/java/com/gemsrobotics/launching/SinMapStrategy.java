package com.gemsrobotics.launching;

import com.gemsrobotics.subsystems.superstructure.Hood;
import edu.wpi.first.math.geometry.Rotation2d;

// represents our shots as vectors. their magnitude is the velocity, and their angle is the hood angle
// the two constants determine how heavily the "up" and "forward" components of the vector scale with range
public final class SinMapStrategy extends LaunchStrategy {
	public static final double RPS_PER_METER_UP = 0.5;
	public static final double RPS_OFFSET_UP = 60.0;
	public static final double RPS_PER_METER_FORWARD = 3.0;
	public static final double RPS_OFFSET_FORWARD = 2.0;
	// Exit angle of ball (in degrees) when hood is at "0" as measured on video
	public static final Rotation2d HOOD_ANGLE_OFFSET = Hood.MIN_ANGLE;

	// 45 : 30 : 18

	@Override
	protected LauncherParameters unsafeParametersFor(final double rangeMeters) {
		final var x = RPS_OFFSET_FORWARD + RPS_PER_METER_FORWARD * rangeMeters;
		final var y = RPS_OFFSET_UP + RPS_PER_METER_UP * rangeMeters;
		return new LauncherParameters(Rotation2d.fromRadians(Math.atan(x / y)).plus(HOOD_ANGLE_OFFSET), Math.hypot(x, y));
	}

	@Override
	public double getMinRangeMeters() {
		return 2.0;
	}

	@Override
	public double getMaxRangeMeters() {
		return 6.0;
	}
}
