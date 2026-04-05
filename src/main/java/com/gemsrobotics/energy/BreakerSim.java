package com.gemsrobotics.energy;

import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Celsius;

public final class BreakerSim {
	private final double m_ambientTemperature;
	private double m_lastTimestamp;
	private double m_temperature;

	// estimated breaker resistance in ohms
	private static final double BREAKER_RESISTANCE = 0.00075;
	// degrees per watt
	private static final double THERMAL_RESISTANCE = 5.0;
	// joules per degree
	private static final double THERMAL_CAPACITANCE = 12.0;
	// units are inverse-seconds
	private static final double TIME_CONSTANT = THERMAL_CAPACITANCE * THERMAL_RESISTANCE;

	public BreakerSim(final Temperature ambientTemperature) {
		m_lastTimestamp = Double.NaN;
		m_ambientTemperature = ambientTemperature.in(Celsius);
		m_temperature = m_ambientTemperature;
	}

	public BreakerSim() {
		this(Celsius.of(25.0));
	}

	public void update(final double totalCurrentAmps) {
		if (Double.isNaN(m_lastTimestamp)) {
			m_lastTimestamp = Timer.getTimestamp();
			return;
		}

		final double timestamp = Timer.getTimestamp();
		final double dt = timestamp - m_lastTimestamp;
		m_lastTimestamp = timestamp;

		final double internalPower = totalCurrentAmps * totalCurrentAmps * BREAKER_RESISTANCE;
		final double steadyStateTemperature = m_ambientTemperature + internalPower * THERMAL_RESISTANCE;
		// represent the exponential convergence towards the steady state temperature for this amount of current
		m_temperature = steadyStateTemperature + (m_temperature - steadyStateTemperature) * Math.exp(-dt / TIME_CONSTANT);
	}

	public double getTemperature() {
		return m_temperature;
	}
}
