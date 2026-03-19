package com.gemsrobotics.energy;

public interface PowerTracking {
    String getName();
    double getCurrent();
    double getVoltage();
    default double getPower() {
        return getVoltage() * getCurrent();
    }
}
