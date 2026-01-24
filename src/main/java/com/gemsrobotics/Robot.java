// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final RobotContainer m_robotContainer;
    private final MatchStateTracker m_matchStateTracker;

    private final VoltageOut m_request;
    private final Follower m_followerRequest;
    private final TalonFX m_intakeTop;
    private final TalonFX m_intakeBottom;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_matchStateTracker = new MatchStateTracker();

        m_request = new VoltageOut(0);
        m_followerRequest = new Follower(Constants.CAN.INTAKE_TOP_TRANSLATION, MotorAlignmentValue.Opposed);

        m_intakeTop = new TalonFX(Constants.CAN.INTAKE_TOP_TRANSLATION, Constants.CAN.kAUX_BUS);
        m_intakeBottom = new TalonFX(Constants.CAN.INTAKE_BOTTOM_TRANSLATION, Constants.CAN.kAUX_BUS);
        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.CurrentLimits.StatorCurrentLimit = 60;
        cfg.Voltage.PeakForwardVoltage = 12;
        cfg.Voltage.PeakReverseVoltage = -12;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeTop.getConfigurator().apply(cfg);
        m_intakeBottom.getConfigurator().apply(cfg);

        SmartDashboard.putNumber("test_volts", 0);
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
        m_request.Output = SmartDashboard.getNumber("test_volts", 0);
        m_intakeTop.setControl(m_request);
        m_intakeBottom.setControl(m_followerRequest);
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