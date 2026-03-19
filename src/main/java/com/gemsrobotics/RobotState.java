package com.gemsrobotics;

import com.gemsrobotics.lib.ConcurrentTimeInterpolatableBuffer;
import com.gemsrobotics.vision.PoseEstimate;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public final class RobotState {
	private static final double LOOKBACK_TIME_SECONDS = 1.0;

	private final List<Consumer<PoseEstimate>> m_visionPoseEstimateConsumers;

	// Pose2d(X meters, Y meters, theta Rotation)
	private final ConcurrentTimeInterpolatableBuffer<Pose2d> m_fieldToVehicle;
	// units in radians per second
	private final ConcurrentTimeInterpolatableBuffer<Double> m_vehicleAngularVelocity;

	private final AtomicReference<PoseEstimate> m_lastVisionPoseEstimate;
	private final AtomicReference<Double> m_lastVisionPoseEstimateTimestamp;
	private final AtomicReference<ChassisSpeeds> m_recentVehicleRelativeVelocity;
	private final AtomicReference<ChassisSpeeds> m_recentFieldRelativeVelocity;

	public RobotState() {
		m_visionPoseEstimateConsumers = new ArrayList<>();
		m_lastVisionPoseEstimate = new AtomicReference<>(PoseEstimate.NULL);
		m_lastVisionPoseEstimateTimestamp = new AtomicReference<>(0.0);

		m_fieldToVehicle = ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME_SECONDS);
		m_fieldToVehicle.addSample(0.0, Pose2d.kZero);

		m_vehicleAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME_SECONDS);
		m_vehicleAngularVelocity.addSample(0.0, 0.0);
		m_recentVehicleRelativeVelocity = new AtomicReference<>(new ChassisSpeeds());
		m_recentFieldRelativeVelocity = new AtomicReference<>(new ChassisSpeeds());
	}

	public void addPoseEstimateConsumer(final Consumer<PoseEstimate> consumer) {
		m_visionPoseEstimateConsumers.add(consumer);
	}

	public void updatePoseEstimate(final PoseEstimate poseEstimate) {
		m_lastVisionPoseEstimate.set(poseEstimate);
		m_lastVisionPoseEstimateTimestamp.set(poseEstimate.timestampSeconds());
		m_visionPoseEstimateConsumers.forEach(consumer -> consumer.accept(poseEstimate));
	}

	public PoseEstimate getLastVisionPoseEstimate() {
		return m_lastVisionPoseEstimate.get();
	}

	public double getLastVisionPoseEstimateTimestamp() {
		return m_lastVisionPoseEstimateTimestamp.get();
	}

	public void addDriveSample(
			final double timeSeconds,
			final Pose2d driveLocation,
			final double driveAngularVelocity,
			final ChassisSpeeds driveRelativeVelocity
	) {
		m_fieldToVehicle.addSample(timeSeconds, driveLocation);
		m_vehicleAngularVelocity.addSample(timeSeconds, driveAngularVelocity);
		m_recentVehicleRelativeVelocity.set(driveRelativeVelocity);
		m_recentFieldRelativeVelocity.set(ChassisSpeeds.fromRobotRelativeSpeeds(driveRelativeVelocity, driveLocation.getRotation()));
	}

	public Map.Entry<Double, Pose2d> getLatestFieldToVehicle() {
		return m_fieldToVehicle.getLatest();
	}

	public ChassisSpeeds getLatestChassisSpeeds_VehicleRelative() {
		return m_recentVehicleRelativeVelocity.get();
	}

	public ChassisSpeeds getLatestChassisSpeeds_FieldRelative() {
		return m_recentFieldRelativeVelocity.get();
	}

	public Optional<Pose2d> getFieldToVehicle(final double timeSeconds) {
		return m_fieldToVehicle.getSample(timeSeconds);
	}

	public Optional<AngularVelocity> getAngularVelocity(final double timeSeconds) {
		return m_vehicleAngularVelocity.getSample(timeSeconds).map(RadiansPerSecond::of);
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

	private Optional<Double> getMaxAbsValueInRange(
			final ConcurrentTimeInterpolatableBuffer<Double> buffer,
			final double startTime,
			final double endTime
	) {
		var range = buffer.getInternalBuffer().subMap(startTime, endTime).values();
		return range.stream().map(Math::abs).max(Double::compare);
	}

	public Optional<Double> getMaxAbsVehicleAngularVelocity(final double startTime, final double endTime) {
		return getMaxAbsValueInRange(m_vehicleAngularVelocity, startTime, endTime);
	}
}
