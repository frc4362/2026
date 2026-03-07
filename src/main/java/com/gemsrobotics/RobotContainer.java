// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.gemsrobotics.commands.AimAtHub;
import com.gemsrobotics.commands.Autos;
import com.gemsrobotics.launching.Launching;
import com.gemsrobotics.lib.Flywheel;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.sim.RobotVisualizer;
import com.gemsrobotics.subsystems.Lights;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;

import static com.gemsrobotics.Constants.CAN.*;
import static edu.wpi.first.units.Units.*;

public final class RobotContainer {
    private final StatusSignalManager m_signalManager;

    private final CommandXboxController m_joystick;

    private final Superstructure m_superstructure;
    private final RobotState m_robotState;
    private final RobotVisualizer m_visualizer;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Vision m_vision;
    private final Launching m_launchCalculator;
    private final Lights m_lights;
    private final Autos m_autos;

    private final ProjectileManager m_projectileManager;

    public RobotContainer() {
        m_signalManager = new StatusSignalManager();
        m_joystick = new CommandXboxController(0);

        m_visualizer = new RobotVisualizer();
        m_robotState = new RobotState();
        m_drivetrain = TunerConstants.createDrivetrain(m_robotState, m_joystick);
        m_drivetrain.setDefaultCommand(new PilotedDrive(
                m_drivetrain,
                m_joystick.rightBumper(),
                () -> -m_joystick.getLeftY(),
                () -> -m_joystick.getLeftX(),
                () -> -m_joystick.getRightX()));

        m_robotState.addPoseEstimateConsumer(estimate -> {
            if (Constants.Vision.ACCEPT_VISION_MEASUREMENTS) {
                final PoseEstimate correctEstimate;
                if (estimate.variance().get(2, 0) >= Constants.Vision.HIGH_VARIANCE) {
                    // insert the known heading reading
                    // rather than hitting the pose estimator with a heading with a high variance
                    // this prevents spiraling off of the field
                    final Rotation2d newRotation = m_drivetrain.getState().Pose.getRotation();
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

                m_drivetrain.addVisionMeasurement(
                        correctEstimate.fieldToVehicle(),
                        correctEstimate.timestampSeconds(),
                        correctEstimate.variance());
            }
        });

        m_vision = new Vision(m_robotState, () ->
            new Limelight4.Inputs(m_drivetrain.getState().Pose, m_drivetrain.getYawVelocity()));

        m_launchCalculator = new Launching(m_robotState);
        m_superstructure = new Superstructure(
                m_drivetrain,
                new Launcher(makeLowerWheelEast(), makeUpperWheelEast()),
                new Launcher(makeLowerWheelWest(), makeUpperWheelWest()),
                new Hopper(m_signalManager, new TalonFX(SINGULATOR_WEST, kAUX_BUS), new TalonFX(SINGULATOR_EAST, kAUX_BUS)),
                new Uptake(m_signalManager,"uptake", new TalonFX(UPTAKE_EAST, kAUX_BUS), new TalonFX(UPTAKE_WEST, kAUX_BUS)),
                new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager,  new TalonFX(INTAKE_TRANSLATION_LEADER, kAUX_BUS), new TalonFX(INTAKE_TRANSLATION_FOLLOWER, kAUX_BUS), new TalonFX(INTAKE_DEPLOYER, kAUX_BUS)));
        m_lights = null;// new Lights();
        m_autos = new Autos(this);

//        m_joystick.rightTrigger().onTrue(Commands.runOnce(() -> m_superstructure.getHopper().setVelocity(90)));
//        m_joystick.rightTrigger().onFalse(Commands.runOnce(() -> m_superstructure.getHopper().setIdle()));

        m_joystick.rightStick().onTrue(new RunCommand(() -> m_superstructure.setRetractIntake(true)));
        m_joystick.rightStick().onFalse(new RunCommand(() -> m_superstructure.setRetractIntake(false)));

        m_joystick.rightTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.LAUNCHING));
        m_joystick.rightTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_joystick.leftTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.INTAKING));
        m_joystick.leftTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_joystick.povDown().whileTrue(new AimAtHub(m_drivetrain));
//        m_joystick.a().onTrue(m_lights.setJammed());
//        m_joystick.a().onFalse(m_lights.setOff());
        //m_joystick.b().onTrue(m_drivetrain.driveToPose(FieldConstants.Hub.nearFace));

        m_projectileManager = new ProjectileManager(
                m_robotState,
                () -> m_launchCalculator.getLatestLaunchParameters()
                        .map(Launching.Parameters::flywheelSpeed)
                        .map(speed -> MetersPerSecond.of(speed * 2 * Math.PI * Units.inchesToMeters(2)))
                        .orElse(MetersPerSecond.of(0)),
                () -> m_launchCalculator.getLatestLaunchParameters().map(Launching.Parameters::hoodAngle).orElse(Rotation2d.kZero));
    }

    public void periodic() {
        // conspicuously, we don't update Superstructure.
        // This is because it is a Subsystem, so it is updated periodically inside the Scheduler
        m_signalManager.periodic();
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

    private Flywheel makeLowerWheelEast() {
        final TalonFX motor = new TalonFX(LAUNCHER_LOWER_EAST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kS = 4.0;
        cfg.Slot0.kA = 0.01;
        cfg.MotionMagic.MotionMagicAcceleration = 500;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable("launcher_east"),
                "lower_wheel",
                m_signalManager,
                Inches.of(2.0),
                motor
        );
    }

    private Flywheel makeLowerWheelWest() {
        final TalonFX motor = new TalonFX(LAUNCHER_LOWER_WEST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kS = 4.0;
        cfg.Slot0.kA = 0.01;
        cfg.MotionMagic.MotionMagicAcceleration = 500;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable("launcher_west"),
                "lower_wheel",
                m_signalManager,
                Inches.of(2.0),
                motor
        );
    }

    private Flywheel makeUpperWheelEast() {
        final TalonFX motor = new TalonFX(LAUNCHER_UPPER_EAST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0 / 2.5;
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kS = 23.0;
        cfg.Slot0.kA = 1;
        cfg.MotionMagic.MotionMagicAcceleration = 1000;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable("launcher_east"),
                "upper_wheel",
                m_signalManager,
                Inches.of(1.0),
                motor
        );
    }

    private Flywheel makeUpperWheelWest() {
        final TalonFX motor = new TalonFX(LAUNCHER_UPPER_WEST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 100;
        cfg.Feedback.SensorToMechanismRatio = 1.0 / 2.5;
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kS = 23.0;
        cfg.Slot0.kA = 1;
        cfg.MotionMagic.MotionMagicAcceleration = 1000;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                NetworkTableInstance.getDefault().getTable("launcher_west"),
                "upper_wheel",
                m_signalManager,
                Inches.of(1.0),
                motor
        );
    }


    public RobotState getRobotState() {
        return m_robotState;
    }

    public CommandXboxController getPilot() {
        return m_joystick;
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return m_drivetrain;
    }

    public Superstructure getSuperstructure() {
        return m_superstructure;
    }

    public Autos getAutos() {
        return m_autos;
    }
}
