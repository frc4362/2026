package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.Robot;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.launching.LaunchingCalculator;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.BLine.Path;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.gemsrobotics.Constants.LAUNCH_TIME_BEFORE_ACTIVE;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public final class SuperstructureCommands {
	private static final double DISTANCE_TO_NEUTRAL_ZONE = FieldConstants.LinesVertical.neutralZoneNear;
	private static final double DISTANCE_TO_OPPOSING_ALLIANCE_ZONE = FieldConstants.LinesVertical.oppAllianceZone;

	// determines if you should feed or score in hub or do nothing and wait
	public static Command makeLaunchCommand(final CommandSwerveDrivetrain swerve, final Superstructure superstructure, final LaunchingCalculator calculator) {
		return makeLaunchCommand_MatchState(swerve, superstructure, calculator, () -> 0.0, () -> 0.0, () -> 0.0);
	}

	public static Command makeLaunchCommand_MatchState(
			final CommandSwerveDrivetrain swerve,
			final Superstructure superstructure,
			final LaunchingCalculator calculator,
			final DoubleSupplier timeUntilActiveSupplier,
			final DoubleSupplier velocityX,
			final DoubleSupplier velocityY
	) {
		final DoubleSupplier myVelocityX = () -> velocityX.getAsDouble()
				* (DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false) ? -1.0 : 1.0);
		final DoubleSupplier myVelocityY = () -> velocityY.getAsDouble()
				* (DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false) ? -1.0 : 1.0);

		final Supplier<Optional<LaunchingCalculator.Parameters>> parametersSupplier = calculator::getLatestLaunchParameters;
		final AimCommand aimingCommand = new AimCommand(
				swerve,
				() -> parametersSupplier.get().map(LaunchingCalculator.Parameters::vehicleRotation),
				myVelocityX,
				myVelocityY);

		return new SequentialCommandGroup(
				new InstantCommand(() -> superstructure.setAllowedToLaunch(false)),
				superstructure.applyWantedState(Superstructure.SystemState.LAUNCHING),
				aimingCommand.alongWith(new RunCommand(() -> {
							parametersSupplier.get().ifPresent(parameters -> {
								superstructure.setLauncherParameters(parameters);
								final boolean headingOk = aimingCommand.getErrorToGoal().isPresent()
										&& abs(aimingCommand.getErrorToGoal().get().getDegrees()) < parameters.vehicleRotationTolerance().getDegrees();
								final boolean activeOk = parameters.isFeeding() || (timeUntilActiveSupplier.getAsDouble() < LAUNCH_TIME_BEFORE_ACTIVE + parameters.timeOfFlight());
								superstructure.setAllowedToLaunch(parameters.isValid() && superstructure.isReadyToStartLaunching() && headingOk && activeOk);
							});
						})
				));
	}

	public static Command launchUntilEmpty(
			final CommandSwerveDrivetrain swerve,
			final Superstructure superstructure,
			final LaunchingCalculator launchingCalculator
	) {
		final Command deadline;
		if (Robot.isReal()) {
			deadline = new WaitUntilCommand(superstructure.isEitherLaunching)
					.andThen(new WaitCommand(2.0).andThen(new WaitUntilCommand(() -> !superstructure.isEitherLaunching.getAsBoolean())));
		} else {
			deadline = new WaitCommand(4.5);
		}

		return SuperstructureCommands.makeLaunchCommand(swerve, superstructure, launchingCalculator).withDeadline(deadline);
	}

	public static Command blineToPoint(final CommandSwerveDrivetrain swerve, final Pose2d endingPose) {
		final Path pathToPoint = new Path(new Path.Waypoint(endingPose));
		return swerve.getTeleopBlineBuilder().build(pathToPoint);
	}

	public static Command lineUpForBump(final RobotState robotState, final CommandSwerveDrivetrain swerve) {
		return swerve.defer(() -> {
			final Pose2d startingPose = robotState.getLatestFieldToVehicle().getValue();
			final Translation2d endingTranslation = FieldConstants.getClosestPreBumpPosition(startingPose);
			return blineToPoint(swerve, new Pose2d(endingTranslation, startingPose.getRotation()));
		});
	}

	public static Command findAndDriveOverBump(final RobotState robotState, final CommandSwerveDrivetrain swerve, final double velocity, final double duration) {
		return Commands.sequence(
				lineUpForBump(robotState, swerve).onlyIf(() -> !FieldConstants.isReadyToCrossBump(robotState.getLatestFieldToVehicle().getValue())),
				driveOverBump(robotState, swerve, velocity, duration));
	}

	public static Command findAndDriveOverBumpBalls(final RobotState robotState, final CommandSwerveDrivetrain swerve, final double velocity, final double duration) {
		return Commands.sequence(
				lineUpForBump(robotState, swerve).onlyIf(() -> !FieldConstants.isReadyToCrossBump(robotState.getLatestFieldToVehicle().getValue())),
				driveOverBumpBalls(robotState, swerve, true, velocity, duration));
	}

	public static Command waitForBumpCross(final CommandSwerveDrivetrain swerve, final double duration) {
		return Commands.sequence(
				new WaitUntilCommand(() -> !swerve.isFlat.getAsBoolean()),
				new WaitUntilCommand(swerve.isFlat.debounce(duration, Debouncer.DebounceType.kRising)));
	}

	public static Command waitForBumpCrossBalls(final RobotState robotState, final CommandSwerveDrivetrain swerve, final double duration) {
		return Commands.sequence(
				new WaitUntilCommand(() -> !swerve.isFlat.getAsBoolean()),
				new WaitUntilCommand(() -> {
					final boolean inAllianceZone = FieldConstants.isInAllianceZone(robotState.getLatestFieldToVehicle().getValue().getTranslation());
					final boolean seesTags = abs(Timer.getTimestamp() - robotState.getLastVisionPoseEstimateTimestamp()) < 0.25;
					return inAllianceZone && seesTags;
				}));
	}

	// please note this does not stop the drive train
	// rotation3d is in Roll Pitch Yaw
	public static Command driveOverBump(final RobotState robotState, final CommandSwerveDrivetrain swerve, final boolean pointsWheelsFirst, final double velocity, final double duration) {
		final SwerveRequest.FieldCentricFacingAngle request = CommandSwerveDrivetrain.makeAimingRequest();
		return Commands.sequence(
				swerve.runOnce(() -> {
					swerve.setControl(new SwerveRequest.PointWheelsAt()
							.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
							.withModuleDirection(Rotation2d.fromRadians(0)));
				}).onlyIf(() -> pointsWheelsFirst),
				swerve.runOnce(() -> {
					final Pose2d startingPose = robotState.getLatestFieldToVehicle().getValue();
					final Translation2d bumpTarget = FieldConstants.getClosestBump(startingPose.getTranslation());
					final double direction = signum(bumpTarget.getX() - startingPose.getX());
					swerve.setControl(request
							.withVelocityX(velocity * direction)
							.withVelocityY(0.0)
							.withTargetDirection(startingPose.getRotation()));
				}),
				Robot.isReal() ? waitForBumpCross(swerve, duration) : new WaitCommand(0.3));
	}

	// please note this does not stop the drive train
	// rotation3d is in Roll Pitch Yaw
	public static Command driveOverBumpBalls(final RobotState robotState, final CommandSwerveDrivetrain swerve, final boolean pointsWheelsFirst, final double velocity, final double duration) {
		final SwerveRequest.FieldCentricFacingAngle request = CommandSwerveDrivetrain.makeAimingRequest();
		return Commands.sequence(
				swerve.runOnce(() -> {
					swerve.setControl(new SwerveRequest.PointWheelsAt()
							.withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
							.withModuleDirection(Rotation2d.fromRadians(0)));
				}).onlyIf(() -> pointsWheelsFirst),
				swerve.runOnce(() -> {
					final Pose2d startingPose = robotState.getLatestFieldToVehicle().getValue();
					final Translation2d bumpTarget = FieldConstants.getClosestBump(startingPose.getTranslation());
					final double direction = signum(bumpTarget.getX() - startingPose.getX());
					swerve.setControl(request
							.withVelocityX(velocity * direction)
							.withVelocityY(0.0)
							.withTargetDirection(startingPose.getRotation()));
				}),
				Robot.isReal() ? waitForBumpCrossBalls(robotState, swerve, duration) : new WaitCommand(0.3));
	}

	// please note this does not stop the drive train
	// rotation3d is in Roll Pitch Yaw
	public static Command driveOverBump(final RobotState robotState, final CommandSwerveDrivetrain swerve, final double velocity, final double duration) {
		return driveOverBump(robotState, swerve, true, velocity, duration);
	}
}
