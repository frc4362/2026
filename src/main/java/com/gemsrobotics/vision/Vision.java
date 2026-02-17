package com.gemsrobotics.vision;

import com.gemsrobotics.RobotState;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public final class Vision {
    private final RobotState m_robotState;
    private final List<Limelight4> m_cameras;

    public Vision(final RobotState robotState, final Supplier<Limelight4.Inputs> inputSupplier, final Limelight4... limelights) {
        m_robotState = robotState;
        m_cameras = Arrays.asList(limelights);
    }

    public void update() {

    }
}
