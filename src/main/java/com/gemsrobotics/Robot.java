// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static com.gemsrobotics.Constants.CAN.kAUX_BUS;

public class Robot extends TimedRobot {

    private final RobotContainer m_robotContainer;
    private final MatchStateTracker m_matchStateTracker;

    private final TalonFX m_launcherLower, m_launcherUpper, m_uptake;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_matchStateTracker = new MatchStateTracker();
        RobotController.setBrownoutVoltage(5.0);

        m_launcherLower = new TalonFX(Constants.CAN.LAUNCHER_LOWER_EAST, kAUX_BUS);
        m_launcherUpper = new TalonFX(Constants.CAN.LAUNCHER_UPPER_EAST, kAUX_BUS);
        m_uptake = new TalonFX(Constants.CAN.UPTAKE_EAST, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80;
        m_launcherUpper.getConfigurator().apply(cfg);
        m_launcherLower.getConfigurator().apply(cfg);
        m_uptake.getConfigurator().apply(cfg);
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        var joystick = m_robotContainer.getPilot();

        if (joystick.a().getAsBoolean()) {
            m_launcherLower.setControl(new VoltageOut(6.0));
            m_launcherUpper.setControl(new VoltageOut(9.0));
            m_uptake.setControl(new VoltageOut(6));
        } else {
            m_launcherLower.setControl(new CoastOut());
            m_launcherUpper.setControl(new CoastOut());
            m_uptake.setControl(new CoastOut());
        }
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
        m_matchStateTracker.update();
        // m_matchStateTracker.getMatchState();
    }
}