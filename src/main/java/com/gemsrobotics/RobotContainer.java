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
import com.gemsrobotics.subsystems.Lights;
import com.gemsrobotics.subsystems.superstructure.*;
import com.gemsrobotics.subsystems.swerve.Telemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;

import static com.gemsrobotics.Constants.CAN.*;
import static edu.wpi.first.units.Units.*;

public final class RobotContainer {
    private final StatusSignalManager m_signalManager;

    private final CommandXboxController m_joystick;

    private final Superstructure m_superstructure;

    private final RobotState m_robotState;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Lights m_lights;

    private final ProjectileManager m_projectileManager;

    public RobotContainer() {
        m_signalManager = new StatusSignalManager();
        m_joystick = new CommandXboxController(0);

        m_superstructure = new Superstructure(
                new Shooter(m_signalManager, new TalonFX(SHOOTER_WEST, kAUX_BUS), new TalonFX(SHOOTER_EAST, kAUX_BUS)),
                new Hopper(m_signalManager, new TalonFX(HOPPER_NORTH, kAUX_BUS), new TalonFX(HOPPER_SOUTH, kAUX_BUS)),
                new Uptake(m_signalManager, new TalonFX(UPTAKE_LEADER, kAUX_BUS), new TalonFX(UPTAKE_FOLLOWER, kAUX_BUS)),
                new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager, new TalonFX(INTAKE_DEPLOYER, kAUX_BUS), new TalonFX(INTAKE_TOP_TRANSLATION, kAUX_BUS))
        );
        m_lights = new Lights();

        m_joystick.rightTrigger().onTrue(m_superstructure.applyWantedState(Superstructure.SystemState.INTAKING));
        m_joystick.rightTrigger().onFalse(m_superstructure.applyWantedState(Superstructure.SystemState.IDLE));
        m_joystick.a().onTrue(m_lights.setJammed());
        m_joystick.a().onFalse(m_lights.setOff());

        m_robotState = new RobotState();
        m_drivetrain = TunerConstants.createDrivetrain(m_robotState, m_joystick);

        m_projectileManager = new ProjectileManager(
                m_robotState,
                // TODO
                () -> MetersPerSecond.of(0.0),//m_superstructure.getShooter()::getLaunchVelocity,
                () -> Rotation2d.fromDegrees(75));
    }

    public void periodic() {
        m_signalManager.periodic();
//        m_superstructure.periodic();

        if (Robot.isSimulation()) {
            m_projectileManager.updateAll();
            // TODO
//            if (m_superstructure.isLaunching()) {
//                m_projectileManager.attemptSpawn();
//            }
        }
    }

    public RobotState getRobotState() {
        return m_robotState;
    }

    public CommandXboxController getPilot() {
        return m_joystick;
    }
}
