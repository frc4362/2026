package com.gemsrobotics.subsystems.superstructure;

import com.gemsrobotics.lib.Flywheel;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class Launcher {
    private static final Distance WHEEL_CIRCUMFERENCE = Inches.of(2).times(2 * Math.PI);
    public static final double SCRUB_FACTOR = 0.7;

    private final Flywheel m_wheelLower, m_wheelUpper;

    public Launcher(final Flywheel wheelLower, final Flywheel wheelUpper) {
        m_wheelLower = wheelLower;
        m_wheelUpper = wheelUpper;
    }

    public void setLinearVelocity(final double velocity) {
        m_wheelLower.setLinearVelocity(velocity);
        m_wheelUpper.setLinearVelocity(velocity);
    }

    public void setAngularVelocity(final double angularVelocity) {
        m_wheelLower.setAngularVelocity(angularVelocity);
        m_wheelUpper.setAngularVelocity(angularVelocity * 1.5);
    }

    public void setOff() {
        m_wheelLower.setOff();
        m_wheelUpper.setOff();
    }

    public double getAngularVelocity() {
        return m_wheelLower.getAngularVelocity();
    }

    public LinearVelocity getLaunchVelocity() {
        return MetersPerSecond.of(getAngularVelocity() * WHEEL_CIRCUMFERENCE.in(Meters) * SCRUB_FACTOR);
    }

    public boolean atReference() {
        return m_wheelLower.isAtReference() && m_wheelUpper.isAtReference();
    }
}