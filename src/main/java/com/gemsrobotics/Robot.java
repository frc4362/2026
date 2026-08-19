// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public final class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final MatchStateScheduler m_matchStateScheduler;
    private final AutoChooser m_autoChooser;
    private final NetworkTable m_table;
    private final BooleanPublisher m_twoTagsPublisher;

    public Robot() {
//        DataLogManager.start();
        m_matchStateScheduler = new MatchStateScheduler();
        m_robotContainer = new RobotContainer(m_matchStateScheduler);
        m_robotContainer.setWatchdogEpochConsumer(this::addEpoch);
        RobotController.setBrownoutVoltage(4.75);

        m_autoChooser = m_robotContainer.getAutos().getAutoChooser();

        m_table = NetworkTableInstance.getDefault().getTable("robot");
        m_twoTagsPublisher = m_table.getBooleanTopic("two_tags").publish();

        SmartDashboard.putData("AutoChooser", m_autoChooser);
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler().withName("Auto Scheduler"));
    }

    @Override
    public void robotPeriodic() {
        m_matchStateScheduler.update();
        m_matchStateScheduler.logMatchState();
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();

        SmartDashboard.putBoolean("ready for bump cross", FieldConstants.isReadyToCrossBump(m_robotContainer.getRobotState().getLatestFieldToVehicle().getValue()));

        m_twoTagsPublisher.set(m_robotContainer.getRobotState().getLastVisionPoseEstimate().tagCount() > 1);
    }

    @Override
    public void disabledInit() {
        m_robotContainer.configureDisabled();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autoChooser.selectedCommandScheduler().isScheduled()) {
            m_autoChooser.selectedCommandScheduler().cancel();
            m_robotContainer.getSwerve().setControl(new SwerveRequest.Idle());
            CommandScheduler.getInstance().schedule(m_robotContainer.getSuperstructure().setWantedState(Superstructure.SystemState.IDLE));
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}