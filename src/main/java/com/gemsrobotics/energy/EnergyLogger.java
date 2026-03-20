package com.gemsrobotics.energy;

import com.gemsrobotics.Constants;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static java.lang.Math.abs;

public final class EnergyLogger {
    public static final String CURRENTS_TABLE_NAME = "currents";
    public static final String POWERS_TABLE_NAME = "powers";
    public static final String ENERGIES_TABLE_NAME = "energies";

    public static final class PowerSinkLogger {
        private final DoublePublisher m_currentPublisher, m_powerPublisher, m_energyPublisher;

        public PowerSinkLogger(final PowerSink sink, final NetworkTable baseTable) {
            m_currentPublisher = baseTable.getSubTable(CURRENTS_TABLE_NAME).getDoubleTopic(sink.getName()).publish();
            m_powerPublisher = baseTable.getSubTable(POWERS_TABLE_NAME).getDoubleTopic(sink.getName()).publish();
            m_energyPublisher = baseTable.getSubTable(ENERGIES_TABLE_NAME).getDoubleTopic(sink.getName()).publish();
        }

        public void log(final double currentAmps, final double powerWatts, final double energyWattHrs) {
            m_currentPublisher.set(currentAmps);
            m_powerPublisher.set(powerWatts);
            m_energyPublisher.set(energyWattHrs);
        }
    }

    private final NetworkTable m_sinksTable;
    private final Map<String, PowerSinkLogger> m_sinkLoggers;
    private final Map<String, Double> m_sinkCurrents,
            m_sinkPowers, m_sinkEnergies;
    private final DoublePublisher m_batteryVoltagePublisher, m_totalCurrentPublisher,
            m_totalPowerPublisher, m_totalEnergyPublisher;

    private double m_totalCurrentAmps;
    private double m_totalPowerWatts;
    private double m_totalEnergyWattHours;

    private final List<PowerSink> m_powerSinks;

    public EnergyLogger() {
        m_sinkLoggers = new HashMap<>();
        m_sinkCurrents = new HashMap<>();
        m_sinkPowers = new HashMap<>();
        m_sinkEnergies = new HashMap<>();

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable("energy");
        m_sinksTable = myTable.getSubTable("sinks");
        m_batteryVoltagePublisher = myTable.getDoubleTopic("battery_volts").publish();
        m_totalCurrentPublisher = myTable.getDoubleTopic("total_current_amps").publish();
        m_totalPowerPublisher = myTable.getDoubleTopic("total_power_watts").publish();
        m_totalEnergyPublisher = myTable.getDoubleTopic("total_energy_wh").publish();

        m_powerSinks = new ArrayList<>();
        registerPowerSink(new RoborioPowerDraw());
        registerPowerSink(ConstantPowerSinks.Radio);
        registerPowerSink(ConstantPowerSinks.CANivores);
        registerPowerSink(ConstantPowerSinks.SwerveCANcoders);
        registerPowerSink(ConstantPowerSinks.Pigeon);
    }

    // if you are adding a MotorPowerSink, make sure the signals are updated from the StatusSignalManager
    public void registerPowerSink(final PowerSink sink) {
        m_sinkLoggers.put(sink.getName(), new PowerSinkLogger(sink, m_sinksTable));
        m_powerSinks.add(sink);
    }

    public void periodic() {
        m_totalCurrentAmps = 0.0;
        m_totalPowerWatts = 0.0;

        for (final PowerSink sink : m_powerSinks) {
            final double current = sink.getCurrent();
            m_sinkCurrents.put(sink.getName(), current);
            m_totalCurrentAmps += current;
            final double power = sink.getPower();
            m_sinkPowers.put(sink.getName(), power);
            m_totalPowerWatts += power;
            final double newEnergyConsumed = joulesToWattHours(power * Constants.kLoopPeriodSeconds);
            m_totalEnergyWattHours += newEnergyConsumed;
            final double totalSinkEnergy = m_sinkEnergies.merge(sink.getName(), newEnergyConsumed, Double::sum);
            m_sinkLoggers.get(sink.getName()).log(current, power, joulesToWattHours(totalSinkEnergy));
        }

        m_batteryVoltagePublisher.set(RobotController.getBatteryVoltage());
        m_totalCurrentPublisher.set(m_totalCurrentAmps);
        m_totalPowerPublisher.set(m_totalPowerWatts);
        m_totalEnergyPublisher.set(m_totalEnergyWattHours);
    }

    private static double joulesToWattHours(final double joules) {
        return joules / 3600.0;
    }
}
