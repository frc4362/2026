package com.gemsrobotics.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.TalonFX;
import com.gemsrobotics.Constants;
import com.gemsrobotics.energy.CanBusStatusStruct;
import com.gemsrobotics.energy.EnergyLogger;
import com.gemsrobotics.energy.MotorPowerSink;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.*;

import static java.lang.Math.abs;

public final class StatusSignalManager {
    private final List<CANBus> m_canBuses;
    // we need to maintain multiple busses, just so each refresh call includes signals from only one bus
    private final Map<String, StatusSignalCollection> m_signalCollections;
    private final Map<String, StructPublisher<CANBus.CANBusStatus>> m_statusPublishers;
    private final Map<StatusSignal<?>, DoublePublisher> m_publishedSignals;
    private final NetworkTable m_statusTable;
    private final EnergyLogger m_energyLogger;

    public StatusSignalManager() {
        m_canBuses = new ArrayList<>();
        m_signalCollections = new HashMap<>();
        m_statusTable = NetworkTableInstance.getDefault().getTable("can");
        m_statusPublishers = new HashMap<>();
        m_publishedSignals = new HashMap<>();
        m_energyLogger = new EnergyLogger();
    }

    public void periodic() {
        // for each bus we are tracking, refresh all the signals
        // and publish its status to the dashboard
        for (final CANBus bus : m_canBuses) {
            final String busName = bus.getName();
            m_signalCollections.get(busName).refreshAll();
            if (DriverStation.isDisabled()) {
                m_statusPublishers.get(busName).set(bus.getStatus());
            }
        }

        // update all the signal publishers at once
        m_publishedSignals.forEach((signal, publisher) -> {
            publisher.set(signal.getValueAsDouble());
        });

        // and do the energy logging
        m_energyLogger.periodic();
    }

    private void maybeSetupBus(final CANBus bus) {
        if (!m_signalCollections.containsKey(bus.getName())) {
            m_canBuses.add(bus);
            m_signalCollections.put(bus.getName(), new StatusSignalCollection());
            final var publisher = m_statusTable.getStructTopic(bus.getName(), CanBusStatusStruct.struct).publish();
            m_statusPublishers.put(bus.getName(), publisher);
        }
    }

    public void registerPublished(final CANBus bus, final StatusSignal<?> signal, final NetworkTable nt, final String ntTopic) {
        register(bus, signal);
        final DoublePublisher publisher = nt.getDoubleTopic(ntTopic).publish();
        m_publishedSignals.put(signal, publisher);
    }

    public void register(final CANBus bus, final StatusSignal<?>... signals) {
        maybeSetupBus(bus);
        m_signalCollections.get(bus.getName()).addSignals(signals);
    }

    public record PowerTrackingStatusSignals(StatusSignal<Current> supplyCurrentSignal, StatusSignal<Voltage> supplyVoltageSignal) {
    }

    public Map<Integer, PowerTrackingStatusSignals> registerPowerTracking(final CANBus bus, final NetworkTable table, final TalonFX... motors) {
        final List<TalonFX> motorList = Arrays.asList(motors);
        final List<String> names;
        if (motors.length > 1) {
            // give them ids if there's multiple
            names = motorList.stream().map(TalonFX::getDeviceID).map(id -> "motor_" + id).toList();
        } else {
            // otherwise just call it "motor"
            names = List.of("motor");
        }

        return registerPowerTracking(bus, table, names, motorList);
    }

    public Map<Integer, PowerTrackingStatusSignals> registerPowerTracking(final CANBus bus, final NetworkTable table, final List<String> names, final List<TalonFX> motors) {
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
            registerPublished(bus, supplyCurrentSignal, table, prefix + "supply_current_amps");
            final var supplyVoltageSignal = motor.getSupplyVoltage(false);
            registerPublished(bus, supplyVoltageSignal, table, prefix + "supply_voltage");

            ret.put(motor.getDeviceID(), new PowerTrackingStatusSignals(supplyCurrentSignal, supplyVoltageSignal));
            pairs.add(new Pair<>(supplyCurrentSignal, supplyVoltageSignal));
        }

        m_energyLogger.registerPowerSink(new MotorPowerSink(NetworkTable.basenameKey(table.getPath()), pairs));

        return ret;
    }
}