// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import choreo.auto.AutoChooser;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public final class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final MatchStateScheduler m_matchStateScheduler;
    private final AutoChooser m_autoChooser;
    private final NetworkTable m_table;
    private final BooleanPublisher m_twoTagsPublisher;
    private final DoublePublisher m_loopFrequencyPublisher;

    private double m_timestamp;
    private Command m_autonomousCommand;

    public Robot() {
        m_matchStateScheduler = new MatchStateScheduler();
        m_robotContainer = new RobotContainer(m_matchStateScheduler);
        RobotController.setBrownoutVoltage(5.0);

        m_autoChooser = m_robotContainer.getAutos().getAutoChooser();

        m_table = NetworkTableInstance.getDefault().getTable("robot");
        m_twoTagsPublisher = m_table.getBooleanTopic("two_tags").publish();
        m_loopFrequencyPublisher = m_table.getDoubleTopic("hz").publish();

        SmartDashboard.putData("AutoChooser", m_autoChooser);
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler().withName("Auto Scheduler"));

        m_autonomousCommand = Commands.none();
        m_timestamp = Timer.getTimestamp();
    }

    @Override
    public void robotPeriodic() {
        m_matchStateScheduler.update();
        m_matchStateScheduler.logMatchState();
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();

        m_twoTagsPublisher.set(m_robotContainer.getRobotState().getLastVisionPoseEstimate().tagCount() > 1);

        final double newTimestamp = Timer.getTimestamp();
        final double dt = newTimestamp - m_timestamp;
        if (dt > 1e-6) {
            m_loopFrequencyPublisher.set(1.0 / dt);
        }
        m_timestamp = newTimestamp;
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
        FieldConstants.publishPoints();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
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