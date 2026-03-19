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
    private final Map<String, Double> m_subsytemCurrents;
    private final Map<String, Double> m_subsytemPowers;
    private final Map<String, Double> m_subsytemEnergies;

    private final DoublePublisher m_batteryVoltagePublisher, m_totalCurrentPublisher,
            m_totalPowerPublisher, m_totalEnergyPublisher;

    private double m_totalCurrentAmps;
    private double m_totalPowerWatts;
    private double m_totalEnergyJouls;

    private final List<PowerTracking> m_powerSinks;

    public EnergyLogger() {
        m_powerSinks = new ArrayList<>();
        m_powerSinks.add(new RoborioPowerDraw());
        m_powerSinks.add(ConstantPowerDraws.Radio);
        m_powerSinks.add(ConstantPowerDraws.CANivores);
        m_powerSinks.add(ConstantPowerDraws.SwerveCANcoders);
        m_powerSinks.add(ConstantPowerDraws.Pigeon);

        m_subsytemCurrents = new HashMap<>();
        m_subsytemPowers = new HashMap<>();
        m_subsytemEnergies = new HashMap<>();

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable("energy");
        m_batteryVoltagePublisher = myTable.getDoubleTopic("battery_volts").publish();
        m_totalCurrentPublisher = myTable.getDoubleTopic("total_current_amps").publish();
        m_totalPowerPublisher = myTable.getDoubleTopic("total_power_watts").publish();
        m_totalEnergyPublisher = myTable.getDoubleTopic("total_energy_wh").publish();
    }

    public void periodic() {
        m_totalCurrentAmps = 0.0;
        m_totalPowerWatts = 0.0;

        for (final PowerTracking sink : m_powerSinks) {
            m_totalCurrentAmps += sink.getCurrent();
            final double power = sink.getPower();
            m_totalPowerWatts += power;
            m_totalEnergyJouls += power * Constants.kLoopPeriodSeconds;
        }

        m_batteryVoltagePublisher.set(RobotController.getBatteryVoltage());
        m_totalCurrentPublisher.set(m_totalCurrentAmps);
        m_totalPowerPublisher.set(m_totalPowerWatts);
        m_totalEnergyPublisher.set(joulesToWattHours(m_totalEnergyJouls));
    }

//    public void reportCurrentUsage(final String subsystemName, final double... amps) {
//        double totalAmps = 0.0;
//        for (final double amp : amps) {
//            totalAmps += abs(amp);
//        }
//
//        final double power = totalAmps * m_batteryVoltage;
//        final double energy = power * 0.02;
//
//        m_totalCurrent += totalAmps;
//        m_totalPower += power;
//        m_totalEnergy += energy;
//
//        m_subsytemCurrents.put(subsystemName, totalAmps);
//        m_subsytemPowers.put(subsystemName, power);
//        m_subsytemEnergies.merge(subsystemName, energy, Double::sum);
//    }

    private static double joulesToWattHours(final double joules) {
        return joules / 3600.0;
    }
}
