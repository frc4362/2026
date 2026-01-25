// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import com.gemsrobotics.subsystems.superstructure.Shooter;
import com.gemsrobotics.subsystems.swerve.Telemetry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public final class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController m_joystick;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest;
    private final FieldCentricEvasion m_driveRequest;
    private final Telemetry m_logger;

    private final StatusSignalManager m_signalManager;
    private final Shooter m_shooter;

    public RobotContainer() {
        m_joystick = new CommandXboxController(0);
        m_drivetrain = TunerConstants.createDrivetrain();
        m_driveRequest = new FieldCentricEvasion(TunerConstants.moduleTranslations, Constants.BUMPER_DEPTH)
                .withDeadband(0.05)
                .withRotationalDeadband(0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withEvading(false);

        m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

        m_signalManager = new StatusSignalManager();
        m_shooter = new Shooter(m_signalManager, Constants.CAN.SHOOTER_LEFT, Constants.CAN.SHOOTER_RIGHT);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(() ->
                        m_driveRequest.withVelocityX(-m_joystick.getLeftY() * MaxSpeed / 4) // Drive forward with negative Y (forward)
                                .withVelocityY(-m_joystick.getLeftX() * MaxSpeed / 4) // Drive left with negative X (left)
                                .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate)));

        m_joystick.rightBumper().onTrue(runOnce(() -> m_shooter.setVelocity(45))); // TODO: runOnce shouldn't be used here
        m_joystick.rightBumper().onFalse(runOnce(m_shooter::setOff));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        m_logger = new Telemetry(MaxSpeed);
        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    public void periodic() {
        m_signalManager.periodic();
        m_shooter.periodic();
    }
}
