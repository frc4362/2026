// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.commands.Autos;
import com.gemsrobotics.commands.SuperstructureCommands;
import com.gemsrobotics.energy.EnergyLogger;
import com.gemsrobotics.launching.LaunchingCalculator;
import com.gemsrobotics.lib.Flywheel;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.sim.RobotVisualizer;
import com.gemsrobotics.subsystems.superstructure.*;
import com.gemsrobotics.commands.PilotedDrive;
import com.gemsrobotics.vision.Limelight4;
import com.gemsrobotics.vision.PoseEstimate;
import com.gemsrobotics.vision.Vision;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;
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
    private final Trigger m_doEarlyAgitationTrigger, m_retractIntakeTrigger;

    public RobotContainer(MatchStateScheduler matchStateScheduler) {
        m_signalManager = new StatusSignalManager();
        m_pilot = new CommandXboxController(Constants.OperatorConstants.kPilotControllerPort);
        m_copilot = new CommandXboxController(Constants.OperatorConstants.kCopilotControllerPort);

        m_matchStateScheduler = matchStateScheduler;
        m_visualizer = new RobotVisualizer();
        m_robotState = new RobotState();
        m_swerve = TunerConstants.createDrivetrain(m_signalManager, m_robotState);
        m_swerve.setDefaultCommand(new PilotedDrive(
                m_swerve,
                m_pilot.rightBumper(),
                () -> -m_pilot.getLeftY(),
                () -> -m_pilot.getLeftX(),
                () -> -m_pilot.getRightX()));

        m_robotState.addPoseEstimateConsumer(estimate -> {
            if (Constants.Vision.ACCEPT_VISION_MEASUREMENTS) {
                final PoseEstimate correctEstimate;
                if (estimate.variance().get(2, 0) >= Constants.Vision.HIGH_VARIANCE || estimate.tagCount() < 2 || DriverStation.isEnabled()) {
                    // insert the known heading reading
                    // rather than hitting the pose estimator with a heading with a high variance
                    // this prevents spiraling off of the field
                    final var poseSample = m_swerve.samplePoseAt(estimate.timestampSeconds());
                    if (poseSample.isEmpty()) {
                        return;
                    }
                    
                    final Rotation2d newRotation = poseSample.get().getRotation();
                    final Matrix<N3, N1> correctVariance = estimate.variance().copy();
                    correctVariance.set(2, 0, 0.0);
                    correctEstimate = new PoseEstimate(
                            estimate.timestampSeconds(),
                            new Pose2d(estimate.fieldToVehicle().getTranslation(), newRotation),
                            correctVariance,
                            estimate.tagCount());
                } else {
                    correctEstimate = estimate;
                }

                m_swerve.addVisionMeasurement(
                        correctEstimate.fieldToVehicle(),
                        correctEstimate.timestampSeconds(),
                        correctEstimate.variance());
            }
        });

        m_vision = new Vision(m_robotState, () ->
            new Limelight4.Inputs(m_swerve.getState().Pose, m_swerve.getYawVelocity()));

        m_launchCalculator = new LaunchingCalculator(m_robotState);
        m_superstructure = new Superstructure(
                m_swerve,
                new Launcher(makeLowerWheel(LAUNCHER_LOWER_EAST, InvertedValue.Clockwise_Positive, "launcher_east"),
                        makeUpperWheel(LAUNCHER_UPPER_EAST, InvertedValue.Clockwise_Positive, "launcher_east")),
                new Launcher(makeLowerWheel(LAUNCHER_LOWER_WEST, InvertedValue.CounterClockwise_Positive, "launcher_west"),
                        makeUpperWheel(LAUNCHER_UPPER_WEST, InvertedValue.CounterClockwise_Positive, "launcher_west")),
                new Hopper(m_signalManager, new TalonFX(SINGULATOR_WEST, kAUX_BUS), new TalonFX(SINGULATOR_EAST, kAUX_BUS)),
                new Uptake(m_signalManager,"uptake", new TalonFX(UPTAKE_EAST, kAUX_BUS), new TalonFX(UPTAKE_WEST, kAUX_BUS)),
                new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager,  new TalonFX(INTAKE_TRANSLATION_LEADER, kAUX_BUS), new TalonFX(INTAKE_TRANSLATION_FOLLOWER, kAUX_BUS), new TalonFX(INTAKE_DEPLOYER, kAUX_BUS)),
                m_robotState);
        m_autos = new Autos(this);

        m_pilot.leftTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.INTAKING));
        m_pilot.leftTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_pilot.a().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.SPITTING));
        m_pilot.a().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

//        m_joystick.rightBumper().whileTrue(new AimAndBrakeCommand(m_drivetrain, () -> Optional.of(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))));
//        m_joystick.rightTrigger().whileTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_launchCalculator));
        m_pilot.rightTrigger().whileTrue(SuperstructureCommands.makeLaunchCommand_MatchState(
                m_swerve,
                m_superstructure,
                m_launchCalculator,
                () -> m_matchStateScheduler.getMatchState().getTimeUntilActive(),
                () -> -m_pilot.getLeftY(),
                () -> -m_pilot.getLeftX()));
        m_pilot.rightTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

//        m_pilot.y().onTrue(SuperstructureCommands.driveOverBump(m_swerve).andThen(m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle()))));

        m_pilot.povRight().whileTrue(m_swerve.applyRequest(() -> {
            return new SwerveRequest.FieldCentric().withVelocityY(-2.75);
        }));

        m_doEarlyAgitationTrigger = new Trigger(DriverStation::isAutonomous).or(m_copilot.a());
        m_retractIntakeTrigger = m_copilot.y();

        m_projectileManager = new ProjectileManager(
                m_robotState,
                () -> m_launchCalculator.getLatestLaunchParameters()
                        .map(LaunchingCalculator.Parameters::flywheelSpeed)
                        .map(speed -> MetersPerSecond.of(speed * 2 * Math.PI * Units.inchesToMeters(2)))
                        .orElse(MetersPerSecond.of(0)),
                () -> m_launchCalculator.getLatestLaunchParameters().map(LaunchingCalculator.Parameters::hoodAngle).orElse(Rotation2d.kZero));
    }

    public void periodic() {
        // Conspicuously, we don't update Superstructure.
        // This is because it is a Subsystem, so it is updated periodically inside the Scheduler
        m_signalManager.periodic();
        m_superstructure.setDoEarlyAgitation(m_doEarlyAgitationTrigger.getAsBoolean());
        m_superstructure.setRetractIntake(m_retractIntakeTrigger.getAsBoolean());
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

    private Flywheel makeLowerWheel(int talonId, InvertedValue invert, String ntTable) {
        final TalonFX motor = new TalonFX(talonId, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 10.0;
        cfg.Slot0.kS = 4.0;
        cfg.Slot0.kA = 2.0;
        cfg.MotionMagic.MotionMagicAcceleration = 700;
        cfg.MotorOutput.Inverted = invert;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable(ntTable).getSubTable("lower_wheel"),
                m_signalManager,
                Inches.of(2.0),
                motor
        );
    }

    private Flywheel makeUpperWheel(int talonId, InvertedValue invert, String ntTable) {
        final TalonFX motor = new TalonFX(talonId, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0 / 2.5;
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kS = 23.0;
        cfg.Slot0.kA = 1.0;
        cfg.MotionMagic.MotionMagicAcceleration = 1000;
        cfg.MotorOutput.Inverted = invert;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable(ntTable).getSubTable("upper_wheel"),
                m_signalManager,
                Inches.of(1.0),
                motor
        );
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
