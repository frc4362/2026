package com.gemsrobotics.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import java.util.HashMap;

public class StatusSignalManager {
    private final HashMap<StatusSignal<?>, DoublePublisher> m_publishedSignals;
    private final StatusSignalCollection m_signals;

    public StatusSignalManager() {
        m_signals = new StatusSignalCollection();
        m_publishedSignals = new HashMap<>();
    }

    public void periodic() {
        m_signals.refreshAll();
        m_publishedSignals.forEach((signal, publisher) -> {
            publisher.set(signal.getValueAsDouble());
        });
    }

    public void registerPublished(StatusSignal<?> signal, NetworkTable nt, String ntTopic) {
        register(signal);

        final DoublePublisher publisher = nt.getDoubleTopic(ntTopic).publish();
        m_publishedSignals.put(signal, publisher);
    }

    public void register(StatusSignal<?>... signals) {
        m_signals.addSignals(signals);
    }
}