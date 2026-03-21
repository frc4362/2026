package com.gemsrobotics.energy;

import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Celsius;

public final class BreakerSim {
	private final double m_ambientTemperature;
	private double m_lastTimestamp;
	private double m_temperature;

	// degrees per watt
	private static final double THERMAL_RESISTANCE = 1.49;
	// joules per degree
	private static final double THERMAL_CAPACITANCE = 10.503;

	public BreakerSim(final Temperature ambientTemperature) {
		m_lastTimestamp = Double.NaN;
		m_ambientTemperature = ambientTemperature.in(Celsius);
		m_temperature = 0.0;
	}

	public BreakerSim() {
		this(Celsius.of(25.0));
	}

	public void update(final double powerInWatts) {
		if (Double.isNaN(m_lastTimestamp)) {
			m_lastTimestamp = Timer.getTimestamp();
		}

		final double timestamp = Timer.getTimestamp();
		final double dt = timestamp - m_lastTimestamp;
		final double cooling = (m_temperature - m_ambientTemperature) / THERMAL_RESISTANCE;
		final double dTdt = (powerInWatts - cooling) / THERMAL_CAPACITANCE;
		m_temperature += dTdt * dt;
	}

	public double getTemperature() {
		return m_temperature;
	}
}
