package com.gemsrobotics.launching;

public abstract class LaunchStrategy {
	protected abstract OldLauncherParameters unsafeParametersFor(final double rangeMeters);
	public abstract double getMinRangeMeters();
	public abstract double getMaxRangeMeters();

	public final OldLauncherParameters getParameters(final double rangeMeters) {
		if (rangeMeters < getMinRangeMeters()) {
			return unsafeParametersFor(getMinRangeMeters());
		} else if (rangeMeters > getMaxRangeMeters()) {
			return unsafeParametersFor(getMaxRangeMeters());
		} else {
			return unsafeParametersFor(rangeMeters);
		}
	}
}
