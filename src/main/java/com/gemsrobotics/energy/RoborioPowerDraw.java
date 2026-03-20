package com.gemsrobotics.energy;

import edu.wpi.first.wpilibj.RobotController;

public class RoborioPowerDraw implements PowerSink {

    @Override
    public String getName() {
        return "roborio";
    }

    @Override
    public double getCurrent() {
        return RobotController.getInputCurrent();
    }

    @Override
    public double getVoltage() {
        return RobotController.getInputVoltage();
    }
}
