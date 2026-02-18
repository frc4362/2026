package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import javax.swing.text.html.Option;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public final class Vision {
    private final RobotState m_robotState;
    private final List<Limelight4> m_cameras;
    private final Supplier<Limelight4.Inputs> m_inputSupplier;

    public Vision(final RobotState robotState, final Supplier<Limelight4.Inputs> inputSupplier, final Limelight4... limelights) {
        m_robotState = robotState;
        m_inputSupplier = inputSupplier;
        m_cameras = Arrays.asList(limelights);
    }

    public void configureCamerasEnabled() {
        m_cameras.forEach(Limelight4::configureEnabled);
    }

    public void configureCamerasDisabled() {
        m_cameras.forEach(Limelight4::configureDisabled);
    }

    public void update() {
        var inputs = m_inputSupplier.get();
        List<Limelight4.Outputs> outputs = m_cameras.stream().map(camera -> camera.update(inputs)).filter(Optional::isPresent).map(Optional::get).toList();

        // TODO here we need to process the pose estimates
        // the gyro-fused single tag estimates, and the megatag estimates
        // can then optionally combine them into one, or just pick which to trust
        // from there, it submits them to robot state



//        m_robotState.updatePoseEstimate
    }

    private PoseEstimate fusePoseEstimates(PoseEstimate a, PoseEstimate b) {
        // TODO
        if (b.timestampSeconds() > a.timestampSeconds()) {
            var temp = a;
            a = b;
            b = temp;
        }

        // the difference in when we took the two measurements
        final Transform2d a2b = m_robotState.getFieldToVehicle(b.timestampSeconds()).get()
                .minus(m_robotState.getFieldToVehicle(a.timestampSeconds()).get());

        return null;
    }

    private Optional<PoseEstimate> processLimelightPoseEstimate(final LimelightPoseEstimateWithVariance poseEstimateWithVariance) {
        final var poseEstimate = poseEstimateWithVariance.mt();
        // if we have already used more recent information, this is not a valid pose estimate
        if (poseEstimate.timestampSeconds <= m_robotState.getLastVisionPoseEstimateTimestamp()) {
            return Optional.empty();
        }

        // if we don't see any tags, this is not a valid pose estimate
        if (poseEstimate.rawFiducials.length < 1) {
            return Optional.empty();
        }

        // some extra things to check if we only have 1 tag
        if (poseEstimate.rawFiducials.length < 2) {
            // do not accept any single-tag estimates which only use one tag which is found to be ambiguous
            for (final var fiducial : poseEstimate.rawFiducials) {
                if (fiducial.ambiguity > Constants.Vision.ACCEPTABLE_AMBIGUITY) {
                    return Optional.empty();
                }
            }

            // maybe check if the yaw of the reading is significantly different from our understood yaw?
        }

        final double quality = poseEstimate.tagCount > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity;
        final double varianceScalar = 1.0 / quality;
        final Matrix<N3, N1> variance = poseEstimateWithVariance.variance();
        final double xyVar = Math.max(variance.get(0, 1), variance.get(1, 1));

        return Optional.of(new PoseEstimate(
                poseEstimate.timestampSeconds,
                poseEstimate.pose,
                quality,
                VecBuilder.fill(xyVar, xyVar, variance.get(0, 2)).times(varianceScalar),
                poseEstimate.tagCount));
    }

    private Optional<PoseEstimate> processGyroFusedPoseEstimate(final LimelightPoseEstimateWithVariance poseEstimateWithVariance) {
        final var poseEstimate = poseEstimateWithVariance.mt();
        // if we have already used more recent information, this is not a valid pose estimate
        if (poseEstimate.timestampSeconds <= m_robotState.getLastVisionPoseEstimateTimestamp()) {
            return Optional.empty();
        }

        // we are looking to use specifically MegaTag 1 readings, which do not suppose a known heading
        if (poseEstimate.isMegaTag2) {
            return Optional.empty();
        }

        // if we don't see any tags, this is not a valid pose estimate
        // if we have more than 1 tag, we should just use MegaTag
        if (poseEstimate.rawFiducials.length != 1) {
            return Optional.empty();
        }

        // if we've recently been turning very fast
        final Optional<Double> maxRecentYawRate = m_robotState.getMaxAbsVehicleAngularVelocity(
                poseEstimate.timestampSeconds - Constants.Vision.YAW_LOOKBACK_SECONDS,
                poseEstimate.timestampSeconds);
        if (maxRecentYawRate.orElse(Double.POSITIVE_INFINITY) > Constants.Vision.HIGH_YAW_RATE.in(RadiansPerSecond)) {
            return Optional.empty();
        }

        // retrieve our "known" pose. if it doesn't exist, we can't fuse with it.
        final Optional<Pose2d> prior = m_robotState.getFieldToVehicle(poseEstimate.timestampSeconds);
        if (prior.isEmpty()) {
            return Optional.empty();
        }

        // retrieve the location of the tag. if it's not part of the field this year, we can't fuse with it.
        int tagId = poseEstimate.rawFiducials[0].id;
        Optional<Pose3d> rawFieldToTag = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);
        if (rawFieldToTag.isEmpty()) {
            return Optional.empty();
        }

        // make the heading zero. this is because the vision is only measuring distance in this case.
        Pose2d fieldToTag = new Pose2d(rawFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);
        // get our vision-derived robot-to-tag measurement.
        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.pose);
        // create the posterior pose, the "most likely" pose estimate, fused with our known gyro reading
        Pose2d posterior = new Pose2d(
                fieldToTag.getTranslation().minus(robotToTag.getTranslation().rotateBy(prior.get().getRotation())),
                prior.get().getRotation());

        final Matrix<N3, N1> variance = poseEstimateWithVariance.variance();
        final double xyVar = Math.max(variance.get(0, 1), variance.get(1, 1));

        return Optional.of(new PoseEstimate(
                poseEstimate.timestampSeconds,
                posterior,
                // assert that this is a high quality measurement
                1.0,
                VecBuilder.fill(xyVar, xyVar, Constants.Vision.HIGH_VARIANCE),
                poseEstimate.rawFiducials.length));
    }
}
