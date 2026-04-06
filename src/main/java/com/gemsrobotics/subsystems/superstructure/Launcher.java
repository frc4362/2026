package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.gemsrobotics.lib.Flywheel;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;

import java.util.List;

import static com.gemsrobotics.Constants.CAN.kAUX_BUS;
import static edu.wpi.first.units.Units.*;

public class Launcher {
    private static final Distance WHEEL_CIRCUMFERENCE = Inches.of(2).times(2 * Math.PI);
    public static final double SCRUB_FACTOR = 0.7;

    private static Flywheel makeLowerWheel(final StatusSignalManager signalManager, int talonId, InvertedValue invert, final NetworkTable baseTable) {
        final TalonFX motor = new TalonFX(talonId, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = false;
        cfg.CurrentLimits.StatorCurrentLimit = 150;
        cfg.CurrentLimits.SupplyCurrentLimit = 70.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 70.0;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 10.0;
        cfg.Slot0.kS = 4.0;
        cfg.Slot0.kA = 2.0;
        cfg.MotionMagic.MotionMagicAcceleration = 2000;
        cfg.MotorOutput.Inverted = invert;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                kAUX_BUS,
                baseTable.getSubTable("lower_wheel"),
                signalManager,
                Inches.of(2.0),
                motor
        );
    }

    private static Flywheel makeUpperWheel(final StatusSignalManager signalManager, final int talonId, InvertedValue invert, final NetworkTable baseTable) {
        final TalonFX motor = new TalonFX(talonId, kAUX_BUS);
        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = false;
        cfg.CurrentLimits.StatorCurrentLimit = 150;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 70.0;
        cfg.Feedback.SensorToMechanismRatio = 1.0 / 2.5;
        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kS = 23.0;
        cfg.Slot0.kA = 0.0;
        cfg.MotionMagic.MotionMagicAcceleration = 2000;
        cfg.MotorOutput.Inverted = invert;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        motor.getConfigurator().apply(cfg);
        return new Flywheel(
                kAUX_BUS,
                baseTable.getSubTable("upper_wheel"),
                signalManager,
                Inches.of(1.0),
                motor
        );
    }

    private final NetworkTable m_table;
    private final Flywheel m_wheelLower, m_wheelUpper;

    public Launcher(final StatusSignalManager signalManager, final String ntName, final int lowerId, final int upperId, final boolean flipMotors) {
        m_table = NetworkTableInstance.getDefault().getTable(ntName);
        final InvertedValue invertedValue = flipMotors ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        m_wheelLower = makeLowerWheel(signalManager, lowerId, invertedValue, m_table);
        m_wheelUpper = makeUpperWheel(signalManager, upperId, invertedValue, m_table);
        signalManager.registerPowerTracking(
                kAUX_BUS,
                m_table,
                m_wheelLower.getMotorLeader(),
                m_wheelUpper.getMotorLeader());
    }

    public void setLinearVelocity(final double velocity) {
        m_wheelLower.setLinearVelocity(velocity);
        m_wheelUpper.setLinearVelocity(velocity);
    }

    public void setAngularVelocity(final double angularVelocity) {
        m_wheelLower.setAngularVelocity(angularVelocity);
        m_wheelUpper.setAngularVelocity(angularVelocity * 1.5);
    }

    public void setOff() {
        m_wheelLower.setOff();
        m_wheelUpper.setOff();
    }

    public double getAngularVelocity() {
        return m_wheelLower.getAngularVelocity();
    }

    public LinearVelocity getLaunchVelocity() {
        return MetersPerSecond.of(getAngularVelocity() * WHEEL_CIRCUMFERENCE.in(Meters) * SCRUB_FACTOR);
    }

    public boolean atReference() {
        return m_wheelLower.isAtReference() && m_wheelUpper.isAtReference();
    }
}