// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.hardware.TalonFX;
import com.gemsrobotics.lib.StatusSignalManager;
import com.gemsrobotics.sim.ProjectileManager;
import com.gemsrobotics.sim.RobotVisualizer;
import com.gemsrobotics.subsystems.Lights;
import com.gemsrobotics.subsystems.superstructure.*;
import com.gemsrobotics.commands.PilotedDrive;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;

import static com.gemsrobotics.Constants.CAN.*;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
        m_robotState = new RobotState();
        m_drivetrain = TunerConstants.createDrivetrain(m_robotState, m_joystick);
        m_drivetrain.setDefaultCommand(new PilotedDrive(
                m_drivetrain,
                m_joystick.rightBumper(),
                () -> -m_joystick.getLeftY(),
                () -> -m_joystick.getLeftX(),
                () -> -m_joystick.getRightX()));

        m_superstructure = new Superstructure(
                m_drivetrain,
                null,//new Launcher(m_signalManager, "left", new TalonFX(LAUNCHER_WEST, kAUX_BUS), new TalonFX(LAUNCHER_EAST, kAUX_BUS)),
                new Hopper(m_signalManager, new TalonFX(SINGULATOR_WEST, kAUX_BUS), new TalonFX(SINGULATOR_EAST, kAUX_BUS)),
                null,//new Uptake(m_signalManager, new TalonFX(UPTAKE_LEADER, kAUX_BUS), new TalonFX(UPTAKE_FOLLOWER, kAUX_BUS)),
                null,//new Hood(m_signalManager, new TalonFX(HOOD, kAUX_BUS)),
                new Intake(m_signalManager,  new TalonFX(INTAKE_TOP_TRANSLATION, kAUX_BUS), new TalonFX(INTAKE_DEPLOYER, kAUX_BUS))
        );
        m_lights =null;// new Lights();

        m_joystick.rightTrigger().onTrue(Commands.runOnce(() -> m_superstructure.getHopper().setVelocity(90)));
        m_joystick.rightTrigger().onFalse(Commands.runOnce(() -> m_superstructure.getHopper().setIdle()));
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

    public RobotState getRobotState() {
        return m_robotState;
    }

    public CommandXboxController getPilot() {
        return m_joystick;
    }
}
