package com.gemsrobotics.energy;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.HashMap;
import java.util.Map;

import static java.lang.Math.abs;

public final class EnergyLogger {
    private final Map<String, Double> m_subsytemCurrents;
    private final Map<String, Double> m_subsytemPowers;
    private final Map<String, Double> m_subsytemEnergies;

    private final DoublePublisher m_batteryVoltagePublisher, m_totalCurrentPublisher,
            m_totalPowerPublisher, m_totalEnergyPublisher;

    private double m_batteryVoltage;
    private double m_totalCurrent;
    private double m_totalPower;
    private double m_totalEnergy;

    public EnergyLogger() {
        m_batteryVoltage = RobotController.getBatteryVoltage();

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
        m_batteryVoltage = RobotController.getBatteryVoltage();

        reportCurrentUsage("roborio", RobotController.getInputCurrent());
        reportCurrentUsage("cancoders", 0.05 * 4);
        reportCurrentUsage("pigeon", 0.04);
        reportCurrentUsage("canivores", 0.03 * 2);
        reportCurrentUsage("radio", 0.5);

        m_batteryVoltagePublisher.set(m_batteryVoltage);
        m_totalCurrentPublisher.set(m_totalCurrent);
        m_totalPowerPublisher.set(m_totalPower);
        m_totalEnergyPublisher.set(m_totalEnergy);
    }

    public void reportCurrentUsage(final String subsystemName, final double... amps) {
        double totalAmps = 0.0;
        for (final double amp : amps) {
            totalAmps += abs(amp);
        }

        final double power = totalAmps * m_batteryVoltage;
        final double energy = power * 0.02;

        m_totalCurrent += totalAmps;
        m_totalPower += power;
        m_totalEnergy += energy;

        m_subsytemCurrents.put(subsystemName, totalAmps);
        m_subsytemPowers.put(subsystemName, power);
        m_subsytemEnergies.merge(subsystemName, energy, Double::sum);
    }

    private static double joulesToWattHours(final double joules) {
        return joules / 3600.0;
    }
}
