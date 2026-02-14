package com.gemsrobotics.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import java.util.HashMap;

public class StatusSignalManager {
    private final HashMap<StatusSignal<?>, DoublePublisher> m_publishedSignals;
    private final StatusSignalCollection m_unpublishedSignals;

    public StatusSignalManager() {
        m_publishedSignals = new HashMap<>();
        m_unpublishedSignals = new StatusSignalCollection();
    }

    public void periodic() {
        // TODO fix this
        m_publishedSignals.forEach((signal, publisher) -> {
            signal.refresh();
            publisher.set(signal.getValueAsDouble());
        });
        m_unpublishedSignals.refreshAll();
    }

    public void registerPublished(StatusSignal<?> signal, NetworkTable nt, String ntTopic) {
        final DoublePublisher publisher = nt.getDoubleTopic(ntTopic).publish();
        m_publishedSignals.put(signal, publisher);
    }

    public void registerUnpublished(StatusSignal<?>... signals) {
        m_unpublishedSignals.addSignals(signals);
    }
}