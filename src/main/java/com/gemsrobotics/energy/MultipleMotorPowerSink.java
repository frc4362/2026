package com.gemsrobotics.energy;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.Arrays;
import java.util.List;

public class MultipleMotorPowerSink implements PowerSink {
    private final String m_name;
    private final List<Pair<StatusSignal<Voltage>, StatusSignal<Current>>> m_supplySignalPairs;

    public MultipleMotorPowerSink(final String name, final Pair<StatusSignal<Voltage>, StatusSignal<Current>>... motors) {
        m_name = name;
        m_supplySignalPairs = Arrays.asList(motors);
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public double getCurrent() {
        double totalCurrent = 0.0;

        for (final var supplyPair : m_supplySignalPairs) {
            totalCurrent += supplyPair.getSecond().getValueAsDouble();
        }

        return totalCurrent;
    }

    @Override
    public double getPower() {
        double totalPower = 0.0;

        for (final var supplyPair : m_supplySignalPairs) {
            totalPower += supplyPair.getFirst().getValueAsDouble();
        }

        return totalPower;
    }
}
