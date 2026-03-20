package com.gemsrobotics.energy;

import edu.wpi.first.wpilibj.RobotController;

import static java.lang.Math.abs;

public enum ConstantPowerSinks implements PowerSink {
    CANcoder("cancoder", 0.05),
    SwerveCANcoders("swerve_cancoders", 0.05 * 4),
    Pigeon("pigeon", 0.04),
    // we always use two of them
    CANivores("canivores", 0.03 * 2),
    Radio("radio", 0.5);

    private final String m_name;
    private final double m_constantDrawAmps;

    ConstantPowerSinks(final String name, final double constantDrawAmps) {
        m_name = name;
        m_constantDrawAmps = constantDrawAmps;
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public double getCurrent() {
        return abs(m_constantDrawAmps);
    }

    @Override
    public double getVoltage() {
        return RobotController.getBatteryVoltage();
    }
}
