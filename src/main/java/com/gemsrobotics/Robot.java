// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gemsrobotics;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private final VoltageOut m_intakeRequest;
    private final Follower m_intakeFollowerRequest;
    private final TalonFX m_intakeTop;
    private final TalonFX m_intakeBottom;
    private final PositionTorqueCurrentFOC m_deployerRequest;
    private final TalonFX m_deployer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_matchStateTracker = new MatchStateTracker();

        m_intakeRequest = new VoltageOut(0);
        m_intakeFollowerRequest = new Follower(Constants.CAN.INTAKE_TOP_TRANSLATION, MotorAlignmentValue.Opposed);

        m_deployerRequest = new PositionTorqueCurrentFOC(0);

        m_intakeTop = new TalonFX(Constants.CAN.INTAKE_TOP_TRANSLATION, Constants.CAN.kAUX_BUS);
        m_intakeBottom = new TalonFX(Constants.CAN.INTAKE_BOTTOM_TRANSLATION, Constants.CAN.kAUX_BUS);
        final var intakecfg = new TalonFXConfiguration();
        intakecfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakecfg.CurrentLimits.StatorCurrentLimit = 60;
        intakecfg.Voltage.PeakForwardVoltage = 12;
        intakecfg.Voltage.PeakReverseVoltage = -12;
        intakecfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeTop.getConfigurator().apply(intakecfg);
        m_intakeBottom.getConfigurator().apply(intakecfg);

        m_deployer = new TalonFX(Constants.CAN.INTAKE_DEPLOYER, Constants.CAN.kAUX_BUS);
        final var deployercfg = new TalonFXConfiguration();
        deployercfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  //may need to be changed
        deployercfg.Feedback.SensorToMechanismRatio = 1.0;  //replace with gearing
        deployercfg.Slot0.kP = 50.0;  //probably will need to be changed
        deployercfg.Slot0.kV = 0.0;
        deployercfg.Slot0.kA = 0.0;
        deployercfg.Slot0.kG = 5.0;   //probably will need to be changed
        deployercfg.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
        deployercfg.TorqueCurrent.PeakReverseTorqueCurrent = -60.0;
        deployercfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_deployer.getConfigurator().apply(deployercfg);

        SmartDashboard.putNumber("intake_test_volts", 0);
        SmartDashboard.putNumber("deployer_test_position", 0);
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
        m_intakeRequest.Output = SmartDashboard.getNumber("intake_test_volts", 0);
        m_intakeTop.setControl(m_intakeRequest);
        m_intakeBottom.setControl(m_intakeFollowerRequest);
        m_deployerRequest.Position = SmartDashboard.getNumber("deployer_test_position", 0);
        m_deployer.setPosition(m_deployerRequest.Position);
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