package com.gemsrobotics.subsystems.superstructure;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Superstructure extends SubsystemBase {
    private static final String NT_KEY = "superstructure";

    public enum SystemState {
        IDLE,
        INTAKING,
        SHOOTING
    }

    private final Shooter m_shooter;
    private final Hopper m_hopper;
    private final Uptake m_uptake;

    private final StringPublisher m_systemStatePublisher;
    private final StringPublisher m_wantedStatePublisher;

    private SystemState m_state;
    private SystemState m_stateWanted;
    private Timer m_stateChangedTimer;
    private boolean m_stateChanged;

    public Superstructure(
            final Shooter shooter,
            final Hopper hopper,
            final Uptake uptake
    ) {
        m_shooter = shooter;
        m_hopper = hopper;
        m_uptake = uptake;

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_wantedStatePublisher = myTable.getStringTopic("wanted_state").publish();
        m_systemStatePublisher = myTable.getStringTopic("system_state").publish();

        m_state = SystemState.IDLE;
        m_stateWanted = SystemState.IDLE;
        m_stateChangedTimer = new Timer();
        m_stateChanged = false;
    }

    @Override
    public void periodic() {
        m_systemStatePublisher.set(m_state.name());
        m_wantedStatePublisher.set(m_stateWanted.name());

        m_shooter.periodic();
        m_hopper.periodic();
        m_uptake.periodic();

        final SystemState newState = switch (m_state) {
            case IDLE -> handleIdle();
            default -> SystemState.IDLE;
        };

        if (newState != m_state) {
            m_state = newState;
            m_stateChangedTimer.reset();
            m_stateChanged = true;
        } else {
            m_stateChanged = false;
        }
    }

    public SystemState handleIdle() {
        m_shooter.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
        return SystemState.IDLE;
    }

    public Command setWantedState(final SystemState state) {
        return runOnce(() -> {
            m_stateWanted = state;
        });
    }

    public Command applyWantedState(final SystemState newState) {
        return run(() -> {
            m_stateWanted = newState;
        }).until(() -> m_state == newState);
    }

    public SystemState getState() {
        return m_state;
    }
}
