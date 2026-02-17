package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public final class Limelight4 {
    public record Inputs(Pose2d robotPose, AngularVelocity rotationRate) {}
    public record Outputs(boolean hasTags, PoseEstimate mt1, PoseEstimate mt2, Inputs captureConditions) {}

    private final String m_name;
    private final NetworkTable m_table;
    private double m_heartbeat;

    public Limelight4(final String name) {
        m_name = name;
        m_table = NetworkTableInstance.getDefault().getTable(m_name);

        m_heartbeat = 0.0;
    }

    public String getName() {
        return m_name;
    }

    public NetworkTable getTable() {
        return m_table;
    }

    public Optional<Outputs> update(final Inputs inputs) {
        double newHeartbeat = LimelightHelpers.getHeartbeat(m_name);
        // no new frame, early exit
        if (newHeartbeat == m_heartbeat) {
            return Optional.empty();
        }

        m_heartbeat = newHeartbeat;

        boolean hasTags = LimelightHelpers.getTV(m_name);

        LimelightHelpers.SetRobotOrientation(
                m_name,
                inputs.robotPose.getRotation().getDegrees(),
                inputs.rotationRate.in(DegreesPerSecond),
                0.0,
                0.0,
                0.0,
                0.0);

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

        return Optional.empty();
//        return Optional.of(new Outputs(hasTags, new PoseEstimate(mt1.timestampSeconds, mt1.pose, ) ));
    }

    public void setCameraPose(final Pose3d cameraPose) {
        LimelightHelpers.setCameraPose_RobotSpace(
                m_name,
                cameraPose.getX(),
                cameraPose.getY(),
                cameraPose.getZ(),
                cameraPose.getRotation().getMeasureX().in(Degrees),
                cameraPose.getRotation().getMeasureY().in(Degrees),
                cameraPose.getRotation().getMeasureZ().in(Degrees));
    }

    public void configureEnabled() {
        // 0.0 is pipeline-controlled downscale
        LimelightHelpers.SetFiducialDownscalingOverride(m_name, 0.0f);
        LimelightHelpers.SetIMUMode(m_name, 4);
        LimelightHelpers.SetIMUAssistAlpha(m_name, Constants.Vision.IMU_ALPHA);
        LimelightHelpers.SetThrottle(m_name, 0);
    }

    public void configureDisabled() {
        LimelightHelpers.SetFiducialDownscalingOverride(m_name, 1.0f);
        LimelightHelpers.SetIMUMode(m_name, 1);
        LimelightHelpers.SetThrottle(m_name, Constants.Vision.DISABLED_THROTTLE);
    }
}
