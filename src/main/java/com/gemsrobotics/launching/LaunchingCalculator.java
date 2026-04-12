package com.gemsrobotics.launching;

import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.lib.math.GeometryUtil;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static java.lang.Math.*;

public final class LaunchingCalculator {
	private static final boolean DO_MOVE_AND_SHOOT = true;
	private static final double FEED_DISTANCE_FROM_ALLIANCE_WALL = 0.5;
	private static final boolean DO_LINEAR_DRAG_COMPENSATION = true;
	private static final double DRAG_CONSTANT_INVERSE_SECONDS = 0.25;
	private static final double TOF_EPSILON = 0.001;
	private static final double MIN_RANGE_METERS_HUB = 1.66;
	private static final double MAX_RANGE_METERS_HUB = 5.5;//4.05;
	private static final double MIN_RANGE_METERS_FEED = 4.87;
	private static final double MAX_RANGE_METERS_FEED = 14.10;
	public static final double LAUNCH_VELOCITY_OFFSET = -0.75;
	public static final Translation2d HUB_CORNER_TO_FEED_LOCKOUT = new Translation2d(Units.Meters.of(0.0), Constants.BALL_STREAM_WIDTH.div(2));

	public record Parameters(
			double timestamp,
			boolean isValid,
			Translation2d target,
			Rotation2d vehicleRotation,
			Rotation2d vehicleRotationTolerance,
			Rotation2d hoodAngle,
			double flywheelSpeed,
			double distance,
			double distanceNoLookahead,
			boolean isFeeding
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

		RANGE_TO_HOOD_ANGLE.put(1.66, Rotation2d.fromDegrees(7.1));
		RANGE_TO_HOOD_ANGLE.put(2.38, Rotation2d.fromDegrees(15.6));
		RANGE_TO_HOOD_ANGLE.put(3.36, Rotation2d.fromDegrees(19.0));
		RANGE_TO_HOOD_ANGLE.put(4.03, Rotation2d.fromDegrees(20.5));
		RANGE_TO_HOOD_ANGLE.put(4.71, Rotation2d.fromDegrees(21.83));
//		RANGE_TO_HOOD_ANGLE.put(5.5, Rotation2d.fromDegrees(23.5));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(5.0, Rotation2d.fromDegrees(30.0));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(6.5, Rotation2d.fromDegrees(30.0));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(8.0, Rotation2d.fromDegrees(30.0));
		RANGE_TO_HOOD_ANGLE_FEEDING.put(17.0, Rotation2d.fromDegrees(30.0));

		RANGE_TO_WHEEL_RPS.put(1.66, 28.0);
		RANGE_TO_WHEEL_RPS.put(2.38, 30.0);
		RANGE_TO_WHEEL_RPS.put(2.67, 31.2);
		RANGE_TO_WHEEL_RPS.put(3.13, 33.76);
		RANGE_TO_WHEEL_RPS.put(3.8, 34.9);
		RANGE_TO_WHEEL_RPS.put(4.03, 36.1);
		RANGE_TO_WHEEL_RPS.put(4.71, 38.1);
//		RANGE_TO_WHEEL_RPS.put(5.5, 38.1);
		RANGE_TO_WHEEL_RPS_FEEDING.put(5.0, 35.0);
		RANGE_TO_WHEEL_RPS_FEEDING.put(6.5, 38.0);
		RANGE_TO_WHEEL_RPS_FEEDING.put(8.0, 42.0);
		RANGE_TO_WHEEL_RPS_FEEDING.put(17.0, 70.0);

		RANGE_TO_TOF_MAP.put(1.66, 0.84);
		RANGE_TO_TOF_MAP.put(2.40, 1.05);
		RANGE_TO_TOF_MAP.put(3.01, 1.0);
		RANGE_TO_TOF_MAP.put(3.18, 1.0);
		RANGE_TO_TOF_MAP.put(4.0, 0.8);
		RANGE_TO_TOF_MAP_FEEDING.put(4.87, 1.07);
		RANGE_TO_TOF_MAP_FEEDING.put(8.3, 1.14);
		RANGE_TO_TOF_MAP_FEEDING.put(10.23, 1.7);
		RANGE_TO_TOF_MAP_FEEDING.put(14.10, 2.47);
	}

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
		// get the most current pose, accounting for phase lag in how it may have changed
		Pose2d currentPose = m_robotState.getLatestFieldToVehicle().getValue();
		final ChassisSpeeds currentVelocity = m_robotState.getLatestChassisSpeeds_FieldRelative();
		currentPose = currentPose.exp(new Twist2d(
				currentVelocity.vxMetersPerSecond * Constants.PHASE_LAG_SECONDS,
				currentVelocity.vyMetersPerSecond * Constants.PHASE_LAG_SECONDS,
				currentVelocity.omegaRadiansPerSecond * Constants.PHASE_LAG_SECONDS));

		final boolean isFeeding = shouldFeed(currentPose);
		final Translation2d target = isFeeding ? getFeedingTarget(currentPose) : getHubTarget();
		final Pose2d launcherStartingPose = currentPose.transformBy(Constants.ROBOT_TO_LAUNCHER);
		final double startingLauncherToTargetDistance = target.getDistance(launcherStartingPose.getTranslation());

		// the velocity of the launcher will not always be the same as the velocity of the robot
		final ChassisSpeeds launcherVelocity = GeometryUtil.transformVelocity(currentVelocity, Constants.ROBOT_TO_LAUNCHER, currentPose.getRotation());

		double tof = getTimeOfFlight(startingLauncherToTargetDistance, isFeeding);
		Pose2d lookaheadLauncherPose = launcherStartingPose;
		double lookaheadLauncherToTargetDistance = startingLauncherToTargetDistance;
		final List<Double> tofDiffs = new ArrayList<>();
		tofDiffs.add(0.0);
		final List<Double> contractionRates = new ArrayList<>(Constants.TOF_RECURSION_LIMIT);

		if (DO_MOVE_AND_SHOOT) {
			// the imparted velocity of the robot times the flight time of the launch
			// clearly, the time of the launch is not the same as when it is taken while still
			// therefore we recurse
			final var impartedVelocity = new Translation2d(launcherVelocity.vxMetersPerSecond, launcherVelocity.vyMetersPerSecond);
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
		// ugly one-liner... think it's the best way around the boxing?
		m_contractionRatePublisher.set(contractionRates.stream().mapToDouble(Double::doubleValue).toArray());

		// when the loop is done, we're stuck with whatever we have converged on after N iterations
		final Pose2d lookaheadRobotPose = lookaheadLauncherPose.transformBy(Constants.ROBOT_TO_LAUNCHER.inverse());
		// TODO if we ever move shooter off center, we need to calculate the heading with that in mind
		final Rotation2d desiredRobotRotation = target.minus(lookaheadRobotPose.getTranslation()).getAngle()
				.rotateBy(Constants.ROBOT_TO_LAUNCHER.getRotation());
		m_lookaheadPosePublisher.set(new Pose2d(lookaheadRobotPose.getTranslation(), desiredRobotRotation));

		final var ret = new Parameters(
				Timer.getTimestamp(),
				isValidLaunchRange(lookaheadLauncherToTargetDistance, isFeeding) && isValidLaunchVelocity(launcherVelocity, isFeeding),
				target,
				desiredRobotRotation,
				getSwerveHeadingTolerance(lookaheadLauncherToTargetDistance, isFeeding),
				getHoodAngle(lookaheadLauncherToTargetDistance, isFeeding),
				getFlywheelVelocity(lookaheadLauncherToTargetDistance, isFeeding) + LAUNCH_VELOCITY_OFFSET,
				lookaheadLauncherToTargetDistance,
				startingLauncherToTargetDistance,
				isFeeding);

		m_launchingParametersPublisher.set(ret);
		m_latestParameters = ret;
	}

	private static final boolean DO_SIMPLE_TOLERANCE = false;
	private static final double FEED_DISTANCE_FROM_SIDE_WALLS = 4.0 / 3.0; // meters
	private static final double SIDEWAYS_ERROR_ALLOWED_HUB = (FieldConstants.Hub.width / 2.0) - Constants.BALL_STREAM_WIDTH.div(2).in(Meters);
	private static final double SIDEWAYS_ERROR_ALLOWED_FEEDING = FEED_DISTANCE_FROM_SIDE_WALLS - Constants.BALL_STREAM_WIDTH.div(2).in(Meters);

	private Rotation2d getSwerveHeadingTolerance(final double virtualDistance, final boolean isFeeding) {
		if (DO_SIMPLE_TOLERANCE) {
			if (isFeeding) {
				return Rotation2d.fromDegrees(6.0);
			} else {
				return Rotation2d.fromDegrees(2.0);
			}
		} else {
			final double sidewaysErrorDistance = isFeeding ? SIDEWAYS_ERROR_ALLOWED_FEEDING : SIDEWAYS_ERROR_ALLOWED_HUB;
			final Translation2d targetA = new Translation2d(virtualDistance, sidewaysErrorDistance);
			final Translation2d targetB = new Translation2d(virtualDistance, -sidewaysErrorDistance);
			final double a = targetA.getSquaredNorm();
			final double b = targetB.getSquaredNorm();
			final double c = sidewaysErrorDistance * 2.0;
			// solve da triangle
			return Rotation2d.fromRadians(acos((a + b - c * c) / (2 * sqrt(a) * sqrt(b)))).div(2.0);
		}
	}

//	private Rotation2d getDriveAngleWithLauncherOffset(final Pose2d vehiclePose, final Translation2d target) {
//		final Rotation2d fieldToTarget = target.minus(vehiclePose.getTranslation()).getAngle();
//
//	}

	public Optional<Parameters> getLatestLaunchParameters() {
		return Optional.ofNullable(m_latestParameters);
	}

	private static double getFlywheelVelocity(final double launcherToTargetDistance, final boolean isFeeding) {
		if (isFeeding) {
			return RANGE_TO_WHEEL_RPS_FEEDING.get(launcherToTargetDistance);
		} else {
			return RANGE_TO_WHEEL_RPS.get(launcherToTargetDistance);
		}
	}

	private static Rotation2d getHoodAngle(final double launcherToTargetDistance, final boolean isFeeding) {
		if (isFeeding) {
			return RANGE_TO_HOOD_ANGLE_FEEDING.get(launcherToTargetDistance);
		} else {
			return RANGE_TO_HOOD_ANGLE.get(launcherToTargetDistance);
		}
	}

	private static double getTimeOfFlight(final double launcherToTargetDistance, final boolean isFeeding) {
		if (isFeeding) {
			return RANGE_TO_TOF_MAP_FEEDING.get(launcherToTargetDistance);
		} else {
			return RANGE_TO_TOF_MAP.get(launcherToTargetDistance);
		}
	}

	private static boolean isValidLaunchRange(final double launcherToTargetDistance, final boolean isFeeding) {
		final double minRangeMeters = isFeeding ? MIN_RANGE_METERS_FEED : MIN_RANGE_METERS_HUB;
		final double maxRangeMeters = isFeeding ? MAX_RANGE_METERS_FEED : MAX_RANGE_METERS_HUB;

		return launcherToTargetDistance < maxRangeMeters && launcherToTargetDistance > minRangeMeters;
	}

	private static boolean isValidLaunchVelocity(final ChassisSpeeds launcherVelocity, final boolean isFeeding) {
		if (DO_MOVE_AND_SHOOT || isFeeding) { // Can feed at any robot velocity
			return true;
		} else {
			return Math.hypot(launcherVelocity.vxMetersPerSecond, launcherVelocity.vyMetersPerSecond) <= 0.25;
		}
	}

	private record FeedingTargetResults(Translation2d target, boolean valid) {
	}

	private static final double FEED_LOCKOUT_VERTEX_DEPTH = Inches.of(90).in(Meters);

	private static Translation2d getFeedingTarget(final Pose2d vehiclePose) {
		final double feedingX = AllianceFlipUtil.applyX(FEED_DISTANCE_FROM_ALLIANCE_WALL);
		final double feedingY = vehiclePose.getTranslation().getY();
		final double clampedFeedingY = MathUtil.clamp(
				feedingY,
				0.0 + FEED_DISTANCE_FROM_SIDE_WALLS,
				FieldConstants.fieldWidth - FEED_DISTANCE_FROM_SIDE_WALLS);

//		final Translation2d leftPoint = AllianceFlipUtil.apply(FieldConstants.Hub.farLeftCorner
//				.plus(HUB_CORNER_TO_FEED_LOCKOUT));
//		final Translation2d rightPoint = AllianceFlipUtil.apply(FieldConstants.Hub.farRightCorner
//				.minus(HUB_CORNER_TO_FEED_LOCKOUT));
//		final Translation2d feedLockoutVertex = leftPoint.interpolate(rightPoint, 0.5)
//				.plus(new Translation2d(AllianceFlipUtil.applyX(FEED_LOCKOUT_VERTEX_DEPTH), 0.0));
//
//		final Translation2dPlus vehicleTranslation = new Translation2dPlus(vehiclePose.getTranslation());
//		if (vehicleTranslation.isWithinAngle(leftPoint, feedLockoutVertex, rightPoint)) {
//			// recognize that we have NO productive feed angle...
//		}

		return new Translation2d(feedingX, clampedFeedingY);
	}

	private static Translation2d getHubTarget() {
		return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
	}

	private static boolean shouldFeed(final Pose2d vehiclePose) {
		return AllianceFlipUtil.applyX(vehiclePose.getTranslation().getX()) > FieldConstants.Hub.farFace.getX();
	}
}
