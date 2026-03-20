package com.gemsrobotics.energy;

public interface PowerSink {
    String getName();
    double getCurrent();
    double getVoltage();
    default double getPower() {
        return getVoltage() * getCurrent();
    }
}
