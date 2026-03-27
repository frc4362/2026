package com.gemsrobotics.commands;

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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.BLine.Path;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.gemsrobotics.Constants.LAUNCH_TIME_BEFORE_ACTIVE;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class SuperstructureCommands {
	private static final double DISTANCE_TO_NEUTRAL_ZONE = FieldConstants.LinesVertical.neutralZoneNear;
	private static final double DISTANCE_TO_OPPOSING_ALLIANCE_ZONE = FieldConstants.LinesVertical.oppAllianceZone;

	private static final Rotation2d LAUNCH_TOLERANCE = Rotation2d.fromDegrees(2.0);

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
		final Supplier<Optional<LaunchingCalculator.Parameters>> parametersSupplier = calculator::getLatestLaunchParameters;
		final AimAndBrakeCommand aimingCommand = new AimAndBrakeCommand(swerve, () -> parametersSupplier.get().map(LaunchingCalculator.Parameters::vehicleRotation), velocityX, velocityY);

		return new SequentialCommandGroup(
				new InstantCommand(() -> superstructure.setAllowedToLaunch(false)),
				superstructure.applyWantedState(Superstructure.SystemState.LAUNCHING),
				aimingCommand.alongWith(new RunCommand(() -> {
							parametersSupplier.get().ifPresent(parameters -> {
								superstructure.setLauncherParameters(parameters);
//								SmartDashboard.putNumber("turning error", aimingCommand.getErrorToGoal().isPresent() ? abs(aimingCommand.getErrorToGoal().get().getDegrees()) : 999.0);
								final var headingOk = aimingCommand.getErrorToGoal().isPresent() && abs(aimingCommand.getErrorToGoal().get().getDegrees()) < LAUNCH_TOLERANCE.getDegrees();
								final boolean activeOk = parameters.isFeeding() || (timeUntilActiveSupplier.getAsDouble() < LAUNCH_TIME_BEFORE_ACTIVE);
								superstructure.setAllowedToLaunch(parameters.isValid() && superstructure.isReadyToLaunch() && headingOk && activeOk);
							});
						})
				));
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

	public static Command findAndDriveOverBump(final RobotState robotState, final CommandSwerveDrivetrain swerve) {
		return Commands.sequence(
				lineUpForBump(robotState, swerve).onlyIf(() -> !FieldConstants.isReadyToCrossBump(robotState.getLatestFieldToVehicle().getValue())),
				driveOverBump(robotState, swerve));
	}

	private static final double BUMP_CROSS_VELOCITY = 3.0;

	public static Command waitForBumpCross(final CommandSwerveDrivetrain swerve) {
		return Commands.sequence(
				new WaitUntilCommand(() -> !swerve.isFlat()),
				new WaitUntilCommand(new Trigger(swerve::isFlat).debounce(0.05, Debouncer.DebounceType.kRising)));
	}

	// please note this does not stop the drive train
	// rotation3d is in Roll Pitch Yaw
	public static Command driveOverBump(final RobotState robotState, final CommandSwerveDrivetrain swerve) {
		final SwerveRequest.FieldCentricFacingAngle request = CommandSwerveDrivetrain.makeAimingRequest();
		if (Robot.isReal()) {
			return Commands.sequence(
				swerve.runOnce(() -> {
					final Pose2d startingPose = robotState.getLatestFieldToVehicle().getValue();
					final Translation2d bumpTarget = FieldConstants.getClosestBump(startingPose.getTranslation());
					final double direction = signum(bumpTarget.getX() - startingPose.getX());
					swerve.setControl(request
							.withVelocityX(BUMP_CROSS_VELOCITY * direction)
							.withVelocityY(0.0)
							.withTargetDirection(startingPose.getRotation()));
				}),
				waitForBumpCross(swerve));
		} else {
			return Commands.sequence(swerve.runOnce(() -> {
				final Pose2d startingPose = robotState.getLatestFieldToVehicle().getValue();
				final Translation2d bumpTarget = FieldConstants.getClosestBump(startingPose.getTranslation());
				final double direction = signum(bumpTarget.getX() - startingPose.getX());
				swerve.setControl(request
						.withVelocityX(BUMP_CROSS_VELOCITY * direction)
						.withVelocityY(0.0)
						.withTargetDirection(startingPose.getRotation()));
			}),
			new WaitCommand(1.0));
		}
	}

	// TODO
	private Rotation2d calculateHeadingTolerance(final double distance) {
		return Rotation2d.kZero;
	}
}
