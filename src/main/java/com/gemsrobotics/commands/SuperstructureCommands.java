package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.launching.LaunchingCalculator;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static com.gemsrobotics.Constants.LAUNCH_TIME_BEFORE_ACTIVE;
import static java.lang.Math.abs;

public class SuperstructureCommands {
	private static final double DISTANCE_TO_NEUTRAL_ZONE = FieldConstants.LinesVertical.neutralZoneNear;
	private static final double DISTANCE_TO_OPPOSING_ALLIANCE_ZONE = FieldConstants.LinesVertical.oppAllianceZone;

	private static final Rotation2d LAUNCH_TOLERANCE = Rotation2d.fromDegrees(2.0);

	// determines if you should feed or score in hub or do nothing and wait
	public static Command makeLaunchCommand(final CommandSwerveDrivetrain swerve, final Superstructure superstructure, final LaunchingCalculator calculator) {
		return makeLaunchCommand_MatchState(swerve, superstructure, calculator, () -> 0.0);
	}

	public static Command makeLaunchCommand_MatchState(
			final CommandSwerveDrivetrain swerve,
			final Superstructure superstructure,
			final LaunchingCalculator calculator,
			final DoubleSupplier timeUntilActiveSupplier
	) {
		final Supplier<Optional<LaunchingCalculator.Parameters>> parametersSupplier = calculator::getLatestLaunchParameters;
		final AimAndBrakeCommand aimingCommand = new AimAndBrakeCommand(swerve, () -> parametersSupplier.get().map(LaunchingCalculator.Parameters::target));

		return new SequentialCommandGroup(
				new InstantCommand(() -> superstructure.setAllowedToLaunch(false)),
				superstructure.applyWantedState(Superstructure.SystemState.LAUNCHING),
				aimingCommand.alongWith(new RunCommand(() -> {
							parametersSupplier.get().ifPresent(parameters -> {
								superstructure.setLauncherParameters(parameters);
								SmartDashboard.putNumber("turning error", aimingCommand.getErrorToGoal().isPresent() ? abs(aimingCommand.getErrorToGoal().get().getDegrees()) : 999.0);
								final var headingOk = aimingCommand.getErrorToGoal().isPresent() && abs(aimingCommand.getErrorToGoal().get().getDegrees()) < LAUNCH_TOLERANCE.getDegrees();
								final boolean activeOk = parameters.isFeeding() || (timeUntilActiveSupplier.getAsDouble() < LAUNCH_TIME_BEFORE_ACTIVE);
								superstructure.setAllowedToLaunch(parameters.isValid() && superstructure.isReadyToLaunch() && headingOk && activeOk);
							});
						})
				));
	}

	// please note this does not stop the drive train
	// rotation3d is in Roll Pitch Yaw
	public static Command driveOverBump(final CommandSwerveDrivetrain swerve) {
		final SwerveRequest.FieldCentricFacingAngle request = CommandSwerveDrivetrain.makeAimingRequest();
		return Commands.sequence(
				swerve.runOnce(() -> {
					final Rotation2d startingHeading = swerve.getState().Pose.getRotation();
					final double velocity = 2.0 * (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? -1.0 : 1.0);
					swerve.setControl(request
							.withVelocityX(velocity)
							.withVelocityY(0.0)
							.withTargetDirection(startingHeading));
				}),
				new WaitUntilCommand(() -> {
					return abs(swerve.getRotation3d().getX()) > 0.25 || abs(swerve.getRotation3d().getY()) > 0.25;
				}),
				new WaitUntilCommand(() -> {
					return abs(swerve.getRotation3d().getX()) < 0.15 || abs(swerve.getRotation3d().getY()) < 0.15;
				}));
	}

	// TODO
	private Rotation2d calculateHeadingTolerance(final double distance) {
		return Rotation2d.kZero;
	}
}
