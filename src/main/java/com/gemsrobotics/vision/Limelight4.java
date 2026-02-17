package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;

import javax.swing.text.html.Option;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public final class Limelight4 {
    public record Inputs(Pose2d robotPose, AngularVelocity rotationRate) {}
    public record Outputs(boolean hasTags, LimelightHelpers.PoseEstimate mt1, LimelightHelpers.PoseEstimate mt2, double[] variance, Inputs captureConditions) {}
    public record LimelightPoseEstimateWithVariance(LimelightHelpers.PoseEstimate mt, Matrix<N3, N1> variance) {}

    private final String m_name;
    private final DoubleArraySubscriber m_varianceTopic;
    private final Transform3d m_robotToCamera;
    private double m_heartbeat;

    public Limelight4(final String name, final Transform3d robotToCamera) {
        m_name = name;
        m_varianceTopic = NetworkTableInstance.getDefault().getTable(m_name).getDoubleArrayTopic("stddevs").subscribe(new double[12]);
        m_robotToCamera = robotToCamera;

        setCameraPose(m_robotToCamera);

        m_heartbeat = 0.0;
    }

    public String getName() {
        return m_name;
    }

    public Optional<Outputs> update(final Inputs inputs) {
        // update robot orientation no matter what
        LimelightHelpers.SetRobotOrientation(
                m_name,
                inputs.robotPose.getRotation().getDegrees(),
                inputs.rotationRate.in(DegreesPerSecond),
                0.0,
                0.0,
                0.0,
                0.0);


        double newHeartbeat = LimelightHelpers.getHeartbeat(m_name);
        // no new frame, early exit
        if (newHeartbeat == m_heartbeat) {
            return Optional.empty();
        }

        m_heartbeat = newHeartbeat;

        // TODO
        boolean hasTags = LimelightHelpers.getTV(m_name);
        double[] currentVariance = m_varianceTopic.get();
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

        return Optional.of(new Outputs(hasTags, mt1, mt2, new double[12], inputs));
    }

    public void setCameraPose(final Transform3d cameraPose) {
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
