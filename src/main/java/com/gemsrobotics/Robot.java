// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private final MatchStateScheduler m_matchStateScheduler;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_matchStateScheduler = new MatchStateScheduler();
        RobotController.setBrownoutVoltage(5.0);
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.configureDisabled();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        m_matchStateScheduler.update();
        SmartDashboard.putString("Match State", m_matchStateScheduler.getMatchState().toString());
    }
}