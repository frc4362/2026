package com.gemsrobotics.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private final Shooter m_shooter;
    private final Hopper m_hopper;
    private final Uptake m_uptake;

    public Superstructure(
            final Shooter shooter,
            final Hopper hopper,
            final Uptake uptake
    ) {
        m_shooter = shooter;
        m_hopper = hopper;
        m_uptake = uptake;
    }

    @Override
    public void periodic() {
        m_shooter.periodic();
        m_hopper.periodic();
        m_uptake.periodic();
    }
}
