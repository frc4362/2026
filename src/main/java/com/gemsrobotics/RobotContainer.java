// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.hardware.TalonFX;
import com.gemsrobotics.commands.Autos;
import com.gemsrobotics.commands.SuperstructureCommands;
import com.gemsrobotics.launching.LaunchingCalculator;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.sim.RobotVisualizer;
import com.gemsrobotics.subsystems.superstructure.*;
import com.gemsrobotics.commands.PilotedDrive;
import com.gemsrobotics.vision.Limelight4;
import com.gemsrobotics.vision.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.SwerveConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.gemsrobotics.Constants.CAN.*;
import static edu.wpi.first.units.Units.*;

public final class RobotContainer {
    private final StatusSignalManager m_signalManager;
    private final CommandXboxController m_pilot, m_copilot;
    private final Superstructure m_superstructure;
    private final MatchStateScheduler m_matchStateScheduler;
    private final RobotState m_robotState;
    private final RobotVisualizer m_visualizer;
    private final CommandSwerveDrivetrain m_swerve;
    private final Vision m_vision;
    private final LaunchingCalculator m_launchCalculator;
    private final Autos m_autos;
    private final ProjectileManager m_projectileManager;
    private final Trigger m_doEarlyAgitationTrigger, m_retractIntakeTrigger, m_wantsIntakingTrigger;

    public RobotContainer(MatchStateScheduler matchStateScheduler) {
        m_signalManager = new StatusSignalManager();
        m_pilot = new CommandXboxController(Constants.OperatorConstants.kPilotControllerPort);
        m_copilot = new CommandXboxController(Constants.OperatorConstants.kCopilotControllerPort);

        final Trigger pilotIntakingTrigger = m_pilot.leftTrigger();
        final Trigger pilotSnakingTrigger = m_pilot.rightBumper();
        m_wantsIntakingTrigger = pilotIntakingTrigger;

        m_matchStateScheduler = matchStateScheduler;
        m_visualizer = new RobotVisualizer();
        m_robotState = new RobotState();
        m_swerve = SwerveConstants.createDrivetrain(m_signalManager, m_robotState);
        m_swerve.setDefaultCommand(new PilotedDrive(
                m_robotState,
                m_swerve,
                () -> -m_pilot.getLeftY(),
                () -> -m_pilot.getLeftX(),
                () -> -m_pilot.getRightX(),
                () -> false,
                pilotIntakingTrigger,
                pilotSnakingTrigger));

        m_robotState.addPoseEstimateConsumer(m_swerve::acceptPoseMeasurement);

        m_vision = new Vision(m_robotState, () ->
            new Limelight4.Inputs(m_swerve.getState().Pose, m_swerve.getYawVelocity()));

        m_launchCalculator = new LaunchingCalculator(m_robotState);
        m_superstructure = new Superstructure(
                m_swerve,
                new Launcher(m_signalManager, "launcher_east", LAUNCHER_LOWER_EAST, LAUNCHER_UPPER_EAST, false),
                new Launcher(m_signalManager, "launcher_west", LAUNCHER_LOWER_WEST, LAUNCHER_UPPER_WEST, true),
                new Hopper(m_signalManager, new TalonFX(SINGULATOR_WEST, kAUX_BUS), new TalonFX(SINGULATOR_EAST, kAUX_BUS)),
                new Uptake(m_signalManager,"uptake", new TalonFX(UPTAKE_EAST, kAUX_BUS), new TalonFX(UPTAKE_WEST, kAUX_BUS)),
                new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager,  new TalonFX(INTAKE_TRANSLATION_LEADER, kAUX_BUS), new TalonFX(INTAKE_TRANSLATION_FOLLOWER, kAUX_BUS), new TalonFX(INTAKE_DEPLOYER, kAUX_BUS)),
                m_robotState);
        m_autos = new Autos(this);

        pilotIntakingTrigger.onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.INTAKING)
                .unless(() -> m_superstructure.getState() == Superstructure.SystemState.LAUNCHING));
        pilotIntakingTrigger.onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_pilot.a().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.SPITTING));
        m_pilot.a().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        final Trigger wantsLaunchTrigger = m_pilot.rightTrigger().or(m_copilot.rightBumper());
        wantsLaunchTrigger.whileTrue(SuperstructureCommands.makeLaunchCommand_MatchState(
                m_swerve,
                m_superstructure,
                m_launchCalculator,
                () -> m_matchStateScheduler.getMatchState().getTimeUntilActive(),
                () -> -m_pilot.getLeftY(),
                () -> -m_pilot.getLeftX()));
        wantsLaunchTrigger.onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_doEarlyAgitationTrigger = new Trigger(DriverStation::isAutonomous).or(m_copilot.a());
        m_retractIntakeTrigger = m_copilot.y();

        m_projectileManager = new ProjectileManager(
                m_robotState,
                () -> m_launchCalculator.getLatestLaunchParameters()
                        .map(LaunchingCalculator.Parameters::flywheelSpeed)
                        .map(speed -> MetersPerSecond.of(speed * 2 * Math.PI * Units.inchesToMeters(2) * 0.75))
                        .orElse(MetersPerSecond.of(0)),
                () -> m_launchCalculator.getLatestLaunchParameters().map(parameters ->
                        Rotation2d.fromDegrees(90).plus(parameters.hoodAngle())).orElse(Rotation2d.kZero));
    }

    public void periodic() {
        // Conspicuously, we don't update Superstructure.
        // This is because it is a Subsystem, so it is updated periodically inside the Scheduler
        m_signalManager.periodic();
        m_superstructure.setDoEarlyAgitation(m_doEarlyAgitationTrigger.getAsBoolean());
        m_superstructure.setRetractIntake(m_retractIntakeTrigger.getAsBoolean());
        m_superstructure.setWantsIntaking(m_wantsIntakingTrigger.getAsBoolean());
        m_vision.update();
        m_launchCalculator.periodic();

        m_visualizer.update(
                m_robotState.getLatestFieldToVehicle().getValue(),
                m_superstructure.getIntakeAngle(),
                m_superstructure.getHoodAngle());

        if (Robot.isSimulation()) {
            m_projectileManager.updateAll();
            if (m_superstructure.isLaunching()) {
                m_projectileManager.attemptSpawn();
            }
        }
    }

    public void configureDisabled() {
        m_vision.configureDisabled();
    }

    public RobotState getRobotState() {
        return m_robotState;
    }

    public LaunchingCalculator getLaunchCalculator() {
        return m_launchCalculator;
    }

    public CommandXboxController getPilot() {
        return m_pilot;
    }

    public CommandSwerveDrivetrain getSwerve() {
        return m_swerve;
    }

    public Superstructure getSuperstructure() {
        return m_superstructure;
    }

    public Autos getAutos() {
        return m_autos;
    }
}
