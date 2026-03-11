package com.gemsrobotics.launching;

import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.lib.math.GeometryUtil;
import com.gemsrobotics.subsystems.superstructure.Hood;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static java.lang.Math.abs;
import static java.lang.Math.exp;

public class LaunchingCalculator {
	public static boolean DO_MOVE_AND_SHOOT = false;
	public static final double FEED_DISTANCE_FROM_WALL = 0.5;

	public record Parameters(
			double timestamp,
			boolean isValid,
			Translation2d target,
			Rotation2d vehicleRotation,
			Rotation2d hoodAngle,
			double flywheelSpeed,
			double distance,
			double distanceNoLookahead,
			boolean isPassing
	) implements StructSerializable, HoodAndRps {
		public static final Struct<Parameters> struct = StructGenerator.genRecord(Parameters.class);

		@Override
		public Rotation2d getHoodAngle() {
			return hoodAngle;
		}

		@Override
		public double getRps() {
			return flywheelSpeed;
		}
	}

	private static final InterpolatingTreeMap<Double, Rotation2d> RANGE_TO_HOOD_ANGLE;
	private static final InterpolatingTreeMap<Double, Rotation2d> RANGE_TO_HOOD_ANGLE_FEEDING;
	private static final InterpolatingDoubleTreeMap RANGE_TO_WHEEL_RPS;
	private static final InterpolatingDoubleTreeMap RANGE_TO_WHEEL_RPS_FEEDING;
	private static final InterpolatingDoubleTreeMap RANGE_TO_TOF_MAP;
	private static final InterpolatingDoubleTreeMap RANGE_TO_TOF_MAP_FEEDING;
	static {
		RANGE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
		RANGE_TO_HOOD_ANGLE_FEEDING = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
		RANGE_TO_WHEEL_RPS = new InterpolatingDoubleTreeMap();
		RANGE_TO_WHEEL_RPS_FEEDING = new InterpolatingDoubleTreeMap();
		RANGE_TO_TOF_MAP = new InterpolatingDoubleTreeMap();
		RANGE_TO_TOF_MAP_FEEDING = new InterpolatingDoubleTreeMap();

		RANGE_TO_HOOD_ANGLE.put(1.6, Rotation2d.fromDegrees(6));
		RANGE_TO_HOOD_ANGLE.put(2.1, Rotation2d.fromDegrees(13));
		RANGE_TO_HOOD_ANGLE.put(2.63, Rotation2d.fromDegrees(19.7));
		RANGE_TO_HOOD_ANGLE.put(3.28, Rotation2d.fromDegrees(21.25));
		RANGE_TO_HOOD_ANGLE.put(3.88, Rotation2d.fromDegrees(24.25));
		RANGE_TO_HOOD_ANGLE.put(4.4, Rotation2d.fromDegrees(27));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(1.0, Rotation2d.fromDegrees(15.0));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(5.0, Rotation2d.fromDegrees(40.0));

		RANGE_TO_WHEEL_RPS.put(1.6, 25.1);
		RANGE_TO_WHEEL_RPS.put(2.1, 28.0);
		RANGE_TO_WHEEL_RPS.put(2.63, 31.0);
		RANGE_TO_WHEEL_RPS.put(3.28, 32.0);
		RANGE_TO_WHEEL_RPS.put(3.88, 34.25);
		RANGE_TO_WHEEL_RPS.put(4.4, 36.25);
		RANGE_TO_WHEEL_RPS_FEEDING.put(1.0, 30.0);
		RANGE_TO_WHEEL_RPS_FEEDING.put(5.0, 40.0);

		RANGE_TO_TOF_MAP.put(1.0, 0.9);
		RANGE_TO_TOF_MAP.put(2.0, 1.0);
		RANGE_TO_TOF_MAP.put(3.0, 1.1);
		RANGE_TO_TOF_MAP.put(4.0, 1.115);
		RANGE_TO_TOF_MAP.put(5.0, 1.2);

		RANGE_TO_TOF_MAP_FEEDING.put(1.0, 0.9);
		RANGE_TO_TOF_MAP_FEEDING.put(2.0, 1.0);
		RANGE_TO_TOF_MAP_FEEDING.put(3.0, 1.1);
		RANGE_TO_TOF_MAP_FEEDING.put(4.0, 1.115);
		RANGE_TO_TOF_MAP_FEEDING.put(5.0, 1.2);
	}

	private static final boolean DO_LINEAR_DRAG_COMPENSATION = false;
	private static final double DRAG_CONSTANT_INVERSE_SECONDS = 0.2;
	private static final double TOF_EPSILON = 0.001;
	private static final double PHASE_LAG_SECONDS = 0.03;
	private static final double MIN_RANGE_METERS = 1.6;
	private static final double MAX_RANGE_METERS = 4.4;

	private final RobotState m_robotState;
	private final StructPublisher<Parameters> m_launchingParametersPublisher;
	private final StructPublisher<Pose2d> m_lookaheadPosePublisher;
	private final DoubleArrayPublisher m_contractionRatePublisher;

	private Parameters m_latestParameters;

	public LaunchingCalculator(final RobotState robotState) {
		m_robotState = robotState;

		final NetworkTable myTable = NetworkTableInstance.getDefault().getTable("launching_calculator");
		m_launchingParametersPublisher = myTable.getStructTopic("parameters", Parameters.struct).publish();
		m_lookaheadPosePublisher = myTable.getStructTopic("lookahead_pose", Pose2d.struct).publish();
		m_contractionRatePublisher = myTable.getDoubleArrayTopic("contraction_rates").publish();

		m_latestParameters = null;
	}

	public void periodic() {
		Pose2d currentPose = m_robotState.getLatestFieldToVehicle().getValue();

		final ChassisSpeeds currentVelocity = m_robotState.getLatestChassisSpeeds_FieldRelative();
		currentPose = currentPose.exp(new Twist2d(
				currentVelocity.vxMetersPerSecond * PHASE_LAG_SECONDS,
				currentVelocity.vyMetersPerSecond * PHASE_LAG_SECONDS,
				currentVelocity.omegaRadiansPerSecond * PHASE_LAG_SECONDS));

		final boolean isFeeding = shouldFeed(currentPose);
		final Translation2d target = isFeeding ? getFeedingTarget(currentPose) : getHubTarget();
		final Pose2d launcherStartingPose = currentPose.transformBy(Constants.ROBOT_TO_LAUNCHER);
		final double startingLauncherToTargetDistance = target.getDistance(launcherStartingPose.getTranslation());

		final Rotation2d currentRotation = currentPose.getRotation();
		// the velocity of the launcher will not always be the same as the velocity of the robot
		final ChassisSpeeds launcherVelocity = GeometryUtil.transformVelocity(currentVelocity, Constants.ROBOT_TO_LAUNCHER, currentRotation);

		double tof = getTimeOfFlight(startingLauncherToTargetDistance, isFeeding);
		Pose2d lookaheadLauncherPose = launcherStartingPose;
		double lookaheadLauncherToTargetDistance = startingLauncherToTargetDistance;
		final List<Double> tofDiffs = new ArrayList<>();
		tofDiffs.add(0.0);
		final List<Double> contractionRates = new ArrayList<>(Constants.TOF_RECURSION_LIMIT);

		if (DO_MOVE_AND_SHOOT) {
			for (int i = 1; i <= Constants.TOF_RECURSION_LIMIT; i++) {
				// calculate new tof and log how much the tof contracted
				final double newTof = getTimeOfFlight(lookaheadLauncherToTargetDistance, isFeeding);
				final double tofDiff = abs(newTof - tof);
				tofDiffs.add(tofDiff);

				double lastTofDiff = tofDiffs.get(i - 1);
				if (lastTofDiff > TOF_EPSILON) {
					contractionRates.add(tofDiff / lastTofDiff);
				} else {
					contractionRates.add(0.0);
				}
				tof = newTof;

				// the imparted velocity of the robot times the flight time of the launch
				// clearly, the time of the launch is not the same as when it is taken while still
				// therefore we recurse
				final var impartedVelocity = new Translation2d(launcherVelocity.vxMetersPerSecond, launcherVelocity.vyMetersPerSecond);

				// reduce our effective tof by the imparted w
				final double effectiveTof;
				if (DO_LINEAR_DRAG_COMPENSATION) {
					effectiveTof = (1 - exp(-DRAG_CONSTANT_INVERSE_SECONDS * tof)) / DRAG_CONSTANT_INVERSE_SECONDS;
				} else {
					effectiveTof = tof;
				}

				// calculate the new pose and distance for the next recursion
				lookaheadLauncherPose = new Pose2d(
						launcherStartingPose.getTranslation().plus(impartedVelocity.times(effectiveTof)),
						launcherStartingPose.getRotation());
				lookaheadLauncherToTargetDistance = target.getDistance(lookaheadLauncherPose.getTranslation());
			}
		}

		// this is all fine to do still if we just skip the "calculate while moving" portion

		// ugly one liner.. think its the best way around the boxing?
		m_contractionRatePublisher.set(contractionRates.stream().mapToDouble(Double::doubleValue).toArray());

		// when the loop is done, we're stuck with whatever we have converged on after N iterations
		final Pose2d lookaheadRobotPose = lookaheadLauncherPose.transformBy(Constants.ROBOT_TO_LAUNCHER.inverse());
		// TODO if we ever move shooter off center, we need to calculate the heading with that in mind
		final Rotation2d desiredRobotRotation = target.minus(lookaheadRobotPose.getTranslation()).getAngle();
		m_lookaheadPosePublisher.set(new Pose2d(lookaheadRobotPose.getTranslation(), desiredRobotRotation));

		final var ret = new Parameters(
				Timer.getTimestamp(),
				isValidLaunchRange(lookaheadLauncherToTargetDistance, isFeeding) && isValidLaunchVelocity(launcherVelocity, isFeeding),
				target,
				desiredRobotRotation,
				getHoodAngle(lookaheadLauncherToTargetDistance, isFeeding),
				getFlywheelVelocity(lookaheadLauncherToTargetDistance, isFeeding),
				lookaheadLauncherToTargetDistance,
				startingLauncherToTargetDistance,
				isFeeding);

		m_launchingParametersPublisher.set(ret);
		m_latestParameters = ret;
	}

	public Optional<Parameters> getLatestLaunchParameters() {
		return Optional.ofNullable(m_latestParameters);
	}

	private double getFlywheelVelocity(final double launcherToTargetDistance, final boolean isFeeding) {
		return RANGE_TO_WHEEL_RPS.get(launcherToTargetDistance);
	}

	private Rotation2d getHoodAngle(final double launcherToTargetDistance, final boolean isFeeding) {
		return RANGE_TO_HOOD_ANGLE.get(launcherToTargetDistance);
	}

	private double getTimeOfFlight(final double launcherToTargetDistance, final boolean isFeeding) {
		return RANGE_TO_TOF_MAP.get(launcherToTargetDistance);
	}

	private boolean isValidLaunchRange(final double launcherToTargetDistance, final boolean isFeeding) {
		return isFeeding || (launcherToTargetDistance < MAX_RANGE_METERS && launcherToTargetDistance > MIN_RANGE_METERS);
	}

	private boolean isValidLaunchVelocity(final ChassisSpeeds launcherVelocity, final boolean isFeeding) {
		if (DO_MOVE_AND_SHOOT) {
			return true;
		} else {
			return Math.hypot(launcherVelocity.vxMetersPerSecond, launcherVelocity.vyMetersPerSecond) <= 0.25;
		}
	}

	private static Translation2d getFeedingTarget(final Pose2d vehiclePose) {
		final double feedingX = AllianceFlipUtil.applyX(FEED_DISTANCE_FROM_WALL);
		final double feedingY = vehiclePose.getTranslation().getY();
		return new Translation2d(feedingX, feedingY);
	}

	private static Translation2d getHubTarget() {
		return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
	}

	private static boolean shouldFeed(final Pose2d vehiclePose) {
		return AllianceFlipUtil.applyX(vehiclePose.getTranslation().getX()) > FieldConstants.Hub.farFace.getX();
	}
}
