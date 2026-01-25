package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.function.DoubleSupplier;

public class Shooter {
    private static final double GEARING = 1.0;

    private final TalonFX m_motorLeft, m_motorRight;

    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final StaticBrake m_coastRequest;
    private final Follower m_followerRequest;

    private final StatusSignal<AngularVelocity> m_leftVelocitySignal, m_rightVelocitySignal;
    private final StatusSignal<Voltage> m_leftVoltsAppliedSignal, m_rightVoltsAppliedSignal;

    private final TalonFXSimState m_leftSimState, m_rightSimState;
    private final FlywheelSim m_flywheelSim;
    private final Notifier m_simNotifier;

    private boolean m_on;

    public Shooter(final StatusSignalManager signalManager, final int leftId, final int rightId) {
        m_motorLeft = new TalonFX(leftId);
        m_motorRight = new TalonFX(rightId);

        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 2.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.MotionMagic.MotionMagicAcceleration = 500.0;
        m_motorLeft.getConfigurator().apply(cfg);
        m_motorRight.getConfigurator().apply(cfg);

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0.0);
        m_request.Slot = 0;
        m_followerRequest = new Follower(leftId, MotorAlignmentValue.Opposed);
        m_coastRequest = new StaticBrake();

        m_on = false;

        // sim code
        m_leftSimState = m_motorLeft.getSimState();
        m_rightSimState = m_motorRight.getSimState();

        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(2);
        m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, .001, GEARING),
                m_motorModel,
                0.01);

        m_simNotifier = new Notifier(this::simulationPeriodic);
        m_simNotifier.startPeriodic(0.02);

        // logging code
        m_leftVelocitySignal = m_motorLeft.getVelocity(false);
        m_leftVoltsAppliedSignal = m_motorLeft.getMotorVoltage(false);
        m_rightVelocitySignal = m_motorRight.getVelocity(false);
        m_rightVoltsAppliedSignal = m_motorRight.getMotorVoltage(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("shooter");
        signalManager.registerPublished(m_leftVelocitySignal, nt, "left_velocity_rps");
        signalManager.registerPublished(m_leftVoltsAppliedSignal, nt, "left_volts");
        signalManager.registerPublished(m_rightVelocitySignal, nt, "right_velocity_rps");
        signalManager.registerPublished(m_rightVoltsAppliedSignal, nt, "right_volts");
    }

    public void setVelocity(final DoubleSupplier velocitySupplier) {
        m_on = true;
        m_request.Velocity = velocitySupplier.getAsDouble();
    }

    public void setVelocity(final double velocity) {
        setVelocity(() -> velocity);
    }

    public void setOff() {
        m_on = false;
    }

    public void periodic() {
        m_motorLeft.setControl(m_on ? m_request : m_coastRequest);
        m_motorRight.setControl(m_followerRequest);
    }

    public void simulationPeriodic() {
        m_leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leftSimState.getMotorVoltage();
        m_flywheelSim.setInputVoltage(voltage);
        m_flywheelSim.update(0.02);

        m_leftSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
        m_rightSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
    }
}