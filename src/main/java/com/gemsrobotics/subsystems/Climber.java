package com.gemsrobotics.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public enum State {
        NONE,
        RETRACTED,
        EXTENDED
    }

    private final StatusSignal<Current> m_ampsSignal;

    public Climber(final StatusSignalManager signalManager, final TalonFX motor) {
        m_ampsSignal = motor.getTorqueCurrent();

        signalManager.registerUnpublished(m_ampsSignal);
    }

    @Override
    public void periodic() {

    }
}
