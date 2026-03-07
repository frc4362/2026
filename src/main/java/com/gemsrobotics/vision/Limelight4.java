package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.html.Option;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public final class Limelight4 {
    private static final String STD_DEVS_KEY = "stddevs";

    public record Inputs(Pose2d robotPose, AngularVelocity rotationRate) {}
    public record Outputs(String cameraName, double heartbeat, boolean hasTags, LimelightPoseEstimateWithVariance mt1, LimelightPoseEstimateWithVariance mt2, Inputs captureConditions) {
        public Optional<LimelightPoseEstimateWithVariance> getBestPoseEstimate() {
            if (mt1.isInvalid() && mt2.isInvalid()) {
                return Optional.empty();
            } else if (mt1.isInvalid()) {
                return Optional.of(mt2);
            } else if (mt2.isValid()) {
                return Optional.of(mt1);
            } else if (mt2.estimate().tagCount >= 2) {
                return Optional.of(mt2);
            } else if (mt1.estimate().tagCount > mt2.estimate().tagCount) {
                return Optional.of(mt1);
            } else {
                return Optional.of(mt2);
            }
        }
    }

    private final String m_name;
    private final DoubleArraySubscriber m_varianceTopic;
    private final Transform3d m_robotToCamera;
    // we want each Limelight to log individually, so pragmatically it is useful for it to own its own logger
    private final VisionProcessingResultsLogger m_logger;
    private double m_heartbeat;

    public Limelight4(final NetworkTable outputsTable, final String name, final Transform3d robotToCamera) {
        m_name = name;
        m_varianceTopic = NetworkTableInstance.getDefault()
                .getTable(m_name)
                .getDoubleArrayTopic(STD_DEVS_KEY)
                .subscribe(new double[12]);
        m_robotToCamera = robotToCamera;

        setCameraPose(m_robotToCamera);

        m_logger = new VisionProcessingResultsLogger(outputsTable, name);
        m_heartbeat = 0.0;
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

        final double[] v = m_varianceTopic.get();
        final Matrix<N3, N1> varianceMt1 = VecBuilder.fill(
                v[Constants.Vision.kMegatag1XStdDevIndex],
                v[Constants.Vision.kMegatag1YStdDevIndex],
                v[Constants.Vision.kMegatag1YawStdDevIndex]);
        final var mt1 = new LimelightPoseEstimateWithVariance(LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name), varianceMt1);
        final Matrix<N3, N1> varianceMt2 = VecBuilder.fill(
                v[Constants.Vision.kMegatag2XStdDevIndex],
                v[Constants.Vision.kMegatag2YStdDevIndex],
                v[Constants.Vision.kMegatag2YawStdDevIndex]);
        final var mt2 = new LimelightPoseEstimateWithVariance(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name), varianceMt2);

        final boolean hasTags = (mt1.estimate().tagCount + mt2.estimate().tagCount) > 0;
        return Optional.of(new Outputs(m_name, m_heartbeat, hasTags, mt1, mt2, inputs));
    }

    private void setCameraPose(final Transform3d cameraPose) {
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
        LimelightHelpers.SetFiducialDownscalingOverride(m_name, 2.0f);
        LimelightHelpers.SetIMUMode(m_name, 1);
        LimelightHelpers.SetThrottle(m_name, Constants.Vision.DISABLED_THROTTLE);
    }

    public VisionProcessingResultsLogger getLogger() {
        return m_logger;
    }
}
