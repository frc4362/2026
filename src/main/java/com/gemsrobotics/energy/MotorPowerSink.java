package com.gemsrobotics.energy;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

public class MotorPowerSink implements PowerSink {
    private final String m_name;
    private final List<StatusSignal<Current>> m_supplyCurrentSignals;
    private final List<StatusSignal<Voltage>> m_supplyVoltageSignals;

    public MotorPowerSink(final String name, final List<Pair<StatusSignal<Current>, StatusSignal<Voltage>>> signalPairs) {
        m_name = name;
        m_supplyCurrentSignals = new ArrayList<>();
        m_supplyVoltageSignals = new ArrayList<>();

        for (final var pair : signalPairs) {
            m_supplyCurrentSignals.add(pair.getFirst());
            m_supplyVoltageSignals.add(pair.getSecond());
        }
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public double getCurrent() {
        double totalCurrent = 0.0;

        for (final var currentSignal : m_supplyCurrentSignals) {
            totalCurrent += abs(currentSignal.getValueAsDouble());
        }

        return totalCurrent;
    }

    @Override
    public double getPower() {
        double totalPower = 0.0;

        // size of the two lists will be the same
        for (int i = 0; i < m_supplyVoltageSignals.size(); i++) {
            totalPower += abs(m_supplyCurrentSignals.get(i).getValueAsDouble()) * m_supplyVoltageSignals.get(i).getValueAsDouble();
        }

        return totalPower;
    }
}
