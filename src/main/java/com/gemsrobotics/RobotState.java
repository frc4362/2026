package com.gemsrobotics;

import com.gemsrobotics.lib.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;

import java.util.Map;
import java.util.Objects;
import java.util.Optional;

// TODO make sure turret frame is only applied once

public class RobotState {
	private static final double LOOKBACK_TIME_SECONDS = 1.0;

	// Pose2d(X meters, Y meters, theta Rotation)
	private final ConcurrentTimeInterpolatableBuffer<Pose2d> m_fieldToVehicle;
	// units in radians per second
	private final ConcurrentTimeInterpolatableBuffer<Double> m_vehicleAngularVelocity;

	private ChassisSpeeds m_recentVehicleRelativeVelocity;
	private ChassisSpeeds m_recentFieldRelativeVelocity;

	public RobotState() {
		m_fieldToVehicle = ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME_SECONDS);
		m_fieldToVehicle.addSample(0.0, Pose2d.kZero);

		m_vehicleAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SECONDS);
		m_vehicleAngularVelocity.addSample(0.0, 0.0);
		m_recentVehicleRelativeVelocity = new ChassisSpeeds();
		m_recentFieldRelativeVelocity = new ChassisSpeeds();
	}

	public void addDriveSample(
			final double timeSeconds,
			final Pose2d driveLocation,
			final double driveAngularVelocity,
			final ChassisSpeeds driveRelativeVelocity
	) {
		m_fieldToVehicle.addSample(timeSeconds, driveLocation);
		m_vehicleAngularVelocity.addSample(timeSeconds, driveAngularVelocity);
		m_recentVehicleRelativeVelocity = driveRelativeVelocity;
		m_recentFieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(driveRelativeVelocity, driveLocation.getRotation());
	}

	public Map.Entry<Double, Pose2d> getLatestFieldToVehicle() {
		return m_fieldToVehicle.getLatest();
	}

	public ChassisSpeeds getLatestChassisSpeeds_VehicleRelative() {
		return m_recentVehicleRelativeVelocity;
	}

	public ChassisSpeeds getLatestChassisSpeeds_FieldRelative() {
		return m_recentFieldRelativeVelocity;
	}

	public Optional<Pose2d> getFieldToVehicle(final double timeSeconds) {
		return m_fieldToVehicle.getSample(timeSeconds);
	}

	public Pose2d getPredictedFieldToVehicle(final double lookaheadTimeSeconds) {
		final Map.Entry<Double, Pose2d> currentFieldToRobot = m_fieldToVehicle.getLatest();
		if (Objects.isNull(currentFieldToRobot)) {
			return Pose2d.kZero;
		}

		// take the velocity per second and multiply it by the seconds looking ahead
		ChassisSpeeds delta = getLatestChassisSpeeds_VehicleRelative();
		delta = delta.times(lookaheadTimeSeconds);
		// integrate the new arc of motion
		return currentFieldToRobot.getValue().exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
	}
}
