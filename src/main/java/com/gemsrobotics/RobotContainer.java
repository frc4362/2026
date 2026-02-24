// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.gemsrobotics.lib.Flywheel;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.sim.RobotVisualizer;
import com.gemsrobotics.subsystems.Lights;
import com.gemsrobotics.subsystems.superstructure.*;
import com.gemsrobotics.commands.PilotedDrive;
import com.gemsrobotics.vision.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final Lights m_lights;

    private final ProjectileManager m_projectileManager;

    public RobotContainer() {
        m_signalManager = new StatusSignalManager();
        m_joystick = new CommandXboxController(0);

        m_visualizer = new RobotVisualizer();
        // TODO make a real consumer
        m_robotState = new RobotState();
        m_drivetrain = TunerConstants.createDrivetrain(m_robotState, m_joystick);
        m_drivetrain.setDefaultCommand(new PilotedDrive(
                m_drivetrain,
                m_joystick.rightBumper(),
                () -> -m_joystick.getLeftY(),
                () -> -m_joystick.getLeftX(),
                () -> -m_joystick.getRightX()));

        m_robotState.addPoseEstimateConsumer(estimate -> {
            final PoseEstimate correctEstimate;
            if (estimate.variance().get(2, 0) >= Constants.Vision.HIGH_VARIANCE) {
                // insert the known heading reading
                // rather than hitting the pose estimator with a heading with a high variance
                // this prevents spiraling off of the field
                final Rotation2d newRotation = m_drivetrain.getState().Pose.getRotation();
                final Matrix<N3, N1> correctVariance = estimate.variance().copy();
                correctVariance.set(2, 0, 0.0);
                estimate.variance().set(2, 0, 0);

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
                    Utils.fpgaToCurrentTime(correctEstimate.timestampSeconds()),
                    correctEstimate.variance());
        });

        m_superstructure = new Superstructure(
                m_drivetrain,
                new Launcher(m_signalManager, "east_launcher", makeLowerWheel(), makeUpperWheel()),
                new Hopper(m_signalManager, new TalonFX(SINGULATOR_WEST, kAUX_BUS), new TalonFX(SINGULATOR_EAST, kAUX_BUS)),
                new Uptake(m_signalManager,"east", new TalonFX(UPTAKE_EAST, kAUX_BUS), new TalonFX(UPTAKE_WEST, kAUX_BUS)),
                null,//new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager,  new TalonFX(INTAKE_TOP_TRANSLATION, kAUX_BUS), new TalonFX(INTAKE_DEPLOYER, kAUX_BUS)));
        m_lights =null;// new Lights();

//        m_joystick.rightTrigger().onTrue(Commands.runOnce(() -> m_superstructure.getHopper().setVelocity(90)));
//        m_joystick.rightTrigger().onFalse(Commands.runOnce(() -> m_superstructure.getHopper().setIdle()));

        m_joystick.rightTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.LAUNCHING));
        m_joystick.rightTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));

        m_joystick.leftTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.INTAKING));
        m_joystick.leftTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));
//        m_joystick.a().onTrue(m_lights.setJammed());
//        m_joystick.a().onFalse(m_lights.setOff());
        //m_joystick.b().onTrue(m_drivetrain.driveToPose(FieldConstants.Hub.nearFace));

        m_projectileManager = new ProjectileManager(
                m_robotState,
                () -> MetersPerSecond.of(0.0),//m_superstructure.getLauncher()::getLaunchVelocity,
                () -> Rotation2d.kZero); //m_superstructure.getHood()::getLaunchAngle);
    }

    public void periodic() {
        m_signalManager.periodic();

        m_visualizer.update(
                m_robotState.getLatestFieldToVehicle().getValue(),
                m_superstructure.getIntakeAngle(),
                m_superstructure.getHoodAngle());

        if (Robot.isSimulation()) {
            m_projectileManager.updateAll();
            // TODO
            if (m_superstructure.isLaunching()) {
                m_projectileManager.attemptSpawn();
            }
        }
    }

    private Flywheel makeLowerWheel() {
        final TalonFX motor = new TalonFX(LAUNCHER_LOWER_EAST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kS = 4.0;
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

    private Flywheel makeUpperWheel() {
        final TalonFX motor = new TalonFX(LAUNCHER_UPPER_EAST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80;
        cfg.Feedback.SensorToMechanismRatio = 1.0 / 2.5;
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kS = 23.0;
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

    public RobotState getRobotState() {
        return m_robotState;
    }

    public CommandXboxController getPilot() {
        return m_joystick;
    }
}
