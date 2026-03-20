package com.gemsrobotics.energy;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static java.lang.Math.abs;

public class MotorPowerSink implements PowerSink {
    protected final String m_name;
    protected final StatusSignal<Current> m_supplyCurrentSignal;
    protected final StatusSignal<Voltage> m_supplyVoltageSignal;

    public MotorPowerSink(final String name, final StatusSignal<Voltage> supplyVoltageSignal, final StatusSignal<Current> supplyCurrentSignal) {
        m_name = name;
        m_supplyVoltageSignal = supplyVoltageSignal;
        m_supplyCurrentSignal = supplyCurrentSignal;
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public double getCurrent() {
        return abs(m_supplyCurrentSignal.getValueAsDouble());
    }

    @Override
    public double getPower() {
        return getCurrent() * m_supplyVoltageSignal.getValueAsDouble();
    }
}
