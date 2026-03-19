package com.gemsrobotics.energy;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class MotorPowerDraw implements PowerTracking {
    private final String m_name;
    private final StatusSignal<Current> m_supplyCurrentSignal;
    private final StatusSignal<Voltage> m_supplyVoltageSignal;

    public MotorPowerDraw(final String name, final StatusSignal<Voltage> supplyVoltageSignal, final StatusSignal<Current> supplyCurrentSignal) {
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
        return m_supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public double getVoltage() {
        return m_supplyVoltageSignal.getValueAsDouble();
    }
}
