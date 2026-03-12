package com.gemsrobotics.commands;

import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.MatchStateScheduler;
import com.gemsrobotics.launching.LauncherParameters;
import com.gemsrobotics.launching.LaunchingCalculator;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
								superstructure.setAllowedToLaunch(parameters.isValid() && superstructure.isReadyToLaunch() && headingOk &&  timeUntilActiveSupplier.getAsDouble() < 1.0);
							});
						})
				));
	}

	private Rotation2d calculateHeadingTolerance(final double distance) {
		return Rotation2d.kZero;
	}
}
