package com.gemsrobotics.shooting;

public interface ShootingStrategy {
	LaunchParameters parametersFor(final double rangeMeters);
	double getMinRangeMeters();
	double getMaxRangeMeters();
}
