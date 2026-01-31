// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {

    private final RobotContainer m_robotContainer;
    private final MatchStateTracker m_matchStateTracker;
    
    private final VelocityTorqueCurrentFOC m_request;
    private final PositionTorqueCurrentFOC m_deployRequest;
    private final CoastOut m_coastRequest;
    private final TalonFX m_intakeTop;
    private final TalonFX m_intakeDeployer;

    Trigger runIntakeTrigger;
    Trigger deployIntakeTrigger;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_matchStateTracker = new MatchStateTracker();
        
        m_request = new VelocityTorqueCurrentFOC(0);
        m_deployRequest = new PositionTorqueCurrentFOC(0);
        m_coastRequest = new CoastOut();

        m_intakeTop = new TalonFX(Constants.CAN.INTAKE_TOP_TRANSLATION, Constants.CAN.kAUX_BUS);
        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.CurrentLimits.StatorCurrentLimit = 120;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Voltage.PeakForwardVoltage = 12;
        cfg.Voltage.PeakReverseVoltage = -12;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kA = 0.0;
        m_intakeTop.getConfigurator().apply(cfg);

        m_intakeDeployer = new TalonFX(Constants.CAN.INTAKE_DEPLOYER, Constants.CAN.kAUX_BUS);
        final var cfgDep = new TalonFXConfiguration();
        cfgDep.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgDep.CurrentLimits.StatorCurrentLimit = 60;
        cfgDep.CurrentLimits.StatorCurrentLimitEnable = true;
        cfgDep.Voltage.PeakForwardVoltage = 12;
        cfgDep.Voltage.PeakReverseVoltage = -12;
        cfgDep.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfgDep.Slot0.kP = 3000;
        cfgDep.Slot0.kD = 30;
        cfgDep.Feedback.SensorToMechanismRatio = 23.0 * (32.0 / 36.0);
        m_intakeDeployer.getConfigurator().apply(cfgDep);

        runIntakeTrigger = m_robotContainer.getPilot().rightBumper();
        deployIntakeTrigger = m_robotContainer.getPilot().leftBumper();
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
        m_intakeTop.setControl(runIntakeTrigger.getAsBoolean() ?
                m_request.withVelocity(90) :
                m_coastRequest);
        m_intakeDeployer.setControl(deployIntakeTrigger.getAsBoolean() ?
                m_deployRequest.withPosition(0) :
                m_coastRequest);
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