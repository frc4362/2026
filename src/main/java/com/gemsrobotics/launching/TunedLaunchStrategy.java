package com.gemsrobotics.launching;

import com.gemsrobotics.subsystems.superstructure.Hood;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

// test launcher parameters tuned from NetworkTables
public final class TunedLaunchStrategy extends LaunchStrategy {
	private static final Rotation2d MIN_ANGLE = Hood.MIN_ANGLE;

	private final DoubleSubscriber m_hoodAngleSubscriber, m_rpsSubscriber;

	public TunedLaunchStrategy(final NetworkTable nt) {
		m_hoodAngleSubscriber = nt.getDoubleTopic("set_hood_angle_degrees").subscribe(Hood.MIN_ANGLE.getDegrees());
		m_rpsSubscriber = nt.getDoubleTopic("set_velocity_rps").subscribe(0.0);

		m_hoodAngleSubscriber.getTopic().publish().set(Hood.MIN_ANGLE.getDegrees());
		m_rpsSubscriber.getTopic().publish().set(0.0);
	}

	@Override
	protected LauncherParameters unsafeParametersFor(final double rangeMetersUnused) {
		return new LauncherParameters(Rotation2d.fromDegrees(m_hoodAngleSubscriber.get()), m_rpsSubscriber.get());
	}

	@Override
	public double getMinRangeMeters() {
		return 0.0;
	}

	@Override
	public double getMaxRangeMeters() {
		// lets be fr
		return 15.0;
	}
}
