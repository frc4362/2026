package com.gemsrobotics.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.TalonFX;
import com.gemsrobotics.energy.EnergyLogger;
import com.gemsrobotics.energy.PowerSink;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import java.util.*;

import static java.lang.Math.abs;

public final class StatusSignalManager {
    private final HashMap<StatusSignal<?>, DoublePublisher> m_publishedSignals;
    private final StatusSignalCollection m_signals;
    private final EnergyLogger m_energyLogger;

    public StatusSignalManager() {
        m_signals = new StatusSignalCollection();
        m_publishedSignals = new HashMap<>();
        m_energyLogger = new EnergyLogger();
    }

    public void periodic() {
        m_signals.refreshAll();
        m_publishedSignals.forEach((signal, publisher) -> {
            publisher.set(signal.getValueAsDouble());
        });

        m_energyLogger.periodic();
    }

    public void registerPublished(StatusSignal<?> signal, NetworkTable nt, String ntTopic) {
        register(signal);

        final DoublePublisher publisher = nt.getDoubleTopic(ntTopic).publish();
        m_publishedSignals.put(signal, publisher);
    }

    public void register(StatusSignal<?>... signals) {
        m_signals.addSignals(signals);
    }

    public record PowerTrackingStatusSignals(StatusSignal<Current> supplyCurrentSignal, StatusSignal<Voltage> supplyVoltageSignal) {
    }

    public Map<Integer, PowerTrackingStatusSignals> registerPowerTracking(final NetworkTable table, final TalonFX... motors) {
        final List<TalonFX> motorList = Arrays.asList(motors);
        final List<String> ids;
        if (motors.length > 1) {
            // give them ids if there's multiple
            ids = motorList.stream().map(TalonFX::getDeviceID).map(id -> "motor_" + id).toList();
        } else {
            // otherwise just call it "motor"
            ids = List.of("motor");
        }

        return registerPowerTracking(table, ids, motorList);
    }

    public Map<Integer, PowerTrackingStatusSignals> registerPowerTracking(final NetworkTable table, final List<String> names, final List<TalonFX> motors) {
        final Map<Integer, PowerTrackingStatusSignals> ret = new HashMap<>(motors.size());
        final List<Pair<StatusSignal<Current>, StatusSignal<Voltage>>> pairs = new ArrayList<>(motors.size());

        for (int i = 0; i < motors.size(); i++) {
            final TalonFX motor = motors.get(i);
            final String prefix;
            if (motors.size() > 1) {
                prefix = names.get(i) + "_";
            } else {
                prefix = "";
            }

            final var supplyCurrentSignal = motor.getSupplyCurrent(false);
            registerPublished(supplyCurrentSignal, table, prefix + "supply_current_amps");
            final var supplyVoltageSignal = motor.getSupplyVoltage(false);
            registerPublished(supplyVoltageSignal, table, prefix + "supply_voltage");

            ret.put(motor.getDeviceID(), new PowerTrackingStatusSignals(supplyCurrentSignal, supplyVoltageSignal));
            pairs.add(new Pair<>(supplyCurrentSignal, supplyVoltageSignal));
        }

        m_energyLogger.registerPowerSink(new MotorPowerSink(NetworkTable.basenameKey(table.getPath()), pairs));

        return ret;
    }

    private class MotorPowerSink implements PowerSink {
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
}