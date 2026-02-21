package com.gemsrobotics.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;

import java.util.Optional;

public final class VisionProcessingResultsLogger {
    private DoublePublisher m_timestampPublisher;
    private BooleanPublisher m_useMegatagPublisher, m_useGyroFusedPublisher;
    private StructPublisher<PoseEstimate> m_megatagEstimatePublisher, m_gyroFusedEstimatePublisher;

    public VisionProcessingResultsLogger(final NetworkTable baseTable, final String cameraName) {
        final NetworkTable myTable = baseTable.getSubTable(cameraName);
        m_timestampPublisher = myTable.getDoubleTopic("timestamp").publish();
        m_megatagEstimatePublisher = myTable.getStructTopic("estimate_megatag", PoseEstimate.struct).publish();
        m_useMegatagPublisher = myTable.getBooleanTopic("use_megatag").publish();
        m_gyroFusedEstimatePublisher = myTable.getStructTopic("estimate_gyro_fused", PoseEstimate.struct).publish();
        m_useGyroFusedPublisher = myTable.getBooleanTopic("use_gyro_fused_estimate").publish();
    }

    public void log(
            final double timestamp,
            final Optional<PoseEstimate> megatagEstimate,
            final Optional<PoseEstimate> gyroFusedEstimate
    ) {
        m_timestampPublisher.set(timestamp);

        m_megatagEstimatePublisher.set(megatagEstimate.orElse(PoseEstimate.NULL));
        m_gyroFusedEstimatePublisher.set(gyroFusedEstimate.orElse(PoseEstimate.NULL));

        m_useMegatagPublisher.set(megatagEstimate.isPresent());
        // only use the gyro-fused estimate if we don't have a megatag estimate
        m_useGyroFusedPublisher.set(gyroFusedEstimate.isPresent() && megatagEstimate.isEmpty());
    }
}
