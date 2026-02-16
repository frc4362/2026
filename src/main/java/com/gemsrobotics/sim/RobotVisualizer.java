package com.gemsrobotics.sim;

import com.gemsrobotics.Constants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructTopic;

import static com.gemsrobotics.Constants.ROBOT_TO_LAUNCHER_Z;
import static edu.wpi.first.units.Units.Inches;

public final  class RobotVisualizer {
    private final StructArrayPublisher<Pose3d> m_robotPublisher;

    private final Transform3d ROBOT_TO_INTAKE = new Transform3d(Inches.of(12.5), Inches.of(0.0), Inches.of(4.0), new Rotation3d());
    private final Transform3d ROBOT_TO_HOOD = new Transform3d(Inches.of(-5.25), Inches.of(0.0), ROBOT_TO_LAUNCHER_Z, new Rotation3d());

    public RobotVisualizer() {
        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(Constants.SIM_VIZ_TABLE_KEY);
        m_robotPublisher = myTable.getStructArrayTopic("robot", Pose3d.struct).publish();
    }

    public void update(final Pose2d robotLocation, final Rotation2d intakeAngle, final Rotation2d hoodAngle) {
        final var robotLocation3d = new Pose3d(robotLocation);
        m_robotPublisher.set(new Pose3d[] {
                new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, intakeAngle.getRadians(), 0.0)),
                new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, hoodAngle.unaryMinus().getRadians(), 0.0))
        });
    }
}
