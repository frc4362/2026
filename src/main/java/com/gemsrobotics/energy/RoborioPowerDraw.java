package com.gemsrobotics.energy;

import edu.wpi.first.wpilibj.RobotController;

public class RoborioPowerDraw implements PowerTracking {

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
