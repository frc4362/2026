package com.gemsrobotics.energy;

import edu.wpi.first.wpilibj.RobotController;

public class RoborioPowerDraw implements PowerSink {

    @Override
    public String getName() {
        return "roborio";
    }

    @Override
    public double getCurrent() {
        return Math.max(0.0, RobotController.getInputCurrent());
    }

    @Override
    public double getPower() {
        return getCurrent() * RobotController.getInputVoltage();
    }
}
