// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.subsystems.superstructure.Hopper;
import com.gemsrobotics.subsystems.superstructure.Shooter;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.superstructure.Uptake;
import com.gemsrobotics.subsystems.swerve.Telemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;

import static com.gemsrobotics.Constants.CAN.*;
import static edu.wpi.first.units.Units.*;

public final class RobotContainer {

    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final StatusSignalManager m_signalManager;

    private final CommandXboxController m_joystick;

    private final Superstructure m_superstructure;

    private final RobotState m_robotState;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final FieldCentricEvasion m_driveRequest;
    private final Telemetry m_logger;

    private final ProjectileManager m_projectileManager;

    public RobotContainer() {
        m_signalManager = new StatusSignalManager();
        m_joystick = new CommandXboxController(0);

        m_superstructure = new Superstructure(
                new Shooter(m_signalManager, new TalonFX(SHOOTER_WEST, kAUX_BUS), new TalonFX(SHOOTER_EAST, kAUX_BUS)),
                new Hopper(m_signalManager, new TalonFX(HOPPER_NORTH, kAUX_BUS), new TalonFX(HOPPER_SOUTH, kAUX_BUS)),
                new Uptake(m_signalManager, new TalonFX(UPTAKE_LEADER, kAUX_BUS), new TalonFX(UPTAKE_FOLLOWER, kAUX_BUS))
        );
        m_joystick.rightTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.SHOOTING));
        m_joystick.rightTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));


        //region drivetrain
        m_robotState = new RobotState();
        m_drivetrain = TunerConstants.createDrivetrain(m_robotState);
        m_driveRequest = new FieldCentricEvasion(TunerConstants.moduleTranslations, Constants.BUMPER_DEPTH)
                .withDeadband(0.05)
                .withRotationalDeadband(0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withEvading(false);
        m_drivetrain.setDefaultCommand(
                m_drivetrain.applyRequest(() ->
                        m_driveRequest.withVelocityX(-m_joystick.getLeftY() * MaxSpeed / 2) // Drive forward with negative Y (forward)
                                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed / 2) // Drive left with negative X (left)
                                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        m_logger = new Telemetry(MaxSpeed);
        m_drivetrain.registerTelemetry(m_logger::telemeterize);
        //endregion

        m_projectileManager = new ProjectileManager(
                m_robotState,
                m_superstructure.getShooter()::getLaunchVelocity,
                () -> Rotation2d.fromDegrees(75));
    }

    public void periodic() {
        m_signalManager.periodic();
        m_superstructure.periodic();

        if (Robot.isSimulation()) {
            m_projectileManager.updateAll();
            if (m_superstructure.isLaunching()) {
                m_projectileManager.attemptSpawn();
            }
        }
    }

    public RobotState getRobotState() {
        return m_robotState;
    }
}
