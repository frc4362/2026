package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import com.gemsrobotics.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public final class Vision {
    private final RobotState m_robotState;
    private final List<Limelight4> m_cameras;
    private final Supplier<Limelight4.Inputs> m_inputSupplier;

    public Vision(final RobotState robotState, final Supplier<Limelight4.Inputs> inputSupplier, final Limelight4... limelights) {
        m_robotState = robotState;
        m_inputSupplier = inputSupplier;
        m_cameras = Arrays.asList(limelights);
    }

    public void update() {
        var inputs = m_inputSupplier.get();
        List<Limelight4.Outputs> outputs = m_cameras.stream().map(camera -> camera.update(inputs)).filter(Optional::isPresent).map(Optional::get).toList();

        // TODO here we need to process the pose estimates
        // the gyro-fused single tag estimates, and the megatag estimates
        // can then optionally combine them into one, or just pick which to trust
        // from there, it submits them to robot state
    }

    private Optional<PoseEstimate> processLimelightPoseEstimate(final LimelightHelpers.PoseEstimate poseEstimate, final double[] variance) {
        if (poseEstimate.rawFiducials.length < 1) {
            return Optional.empty();
        }

        Matrix<N3, N1> varianceMatrix;
        if (poseEstimate.isMegaTag2) {
            varianceMatrix = VecBuilder.fill(variance[Constants.Vision.kMegatag2XStdDevIndex], variance[Constants.Vision.kMegatag2YStdDevIndex], variance[Constants.Vision.kMegatag2YawStdDevIndex]);
        } else {
            varianceMatrix = VecBuilder.fill(variance[Constants.Vision.kMegatag1XStdDevIndex], variance[Constants.Vision.kMegatag1YStdDevIndex], variance[Constants.Vision.kMegatag1YawStdDevIndex]);
        }

        final double quality = poseEstimate.tagCount > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity;

        return Optional.of(new PoseEstimate(
                poseEstimate.timestampSeconds,
                poseEstimate.pose,
                quality,
                varianceMatrix,
                poseEstimate.tagCount));
    }

    private Optional<PoseEstimate> processGyroFusedPoseEstimate(final LimelightHelpers.PoseEstimate poseEstimate, final double[] variance) {
        return Optional.empty();
    }
//
//    private Optional<VisionFieldPoseEstimate> fuseWithGyro(MegatagPoseEstimate poseEstimate) {
//        if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
//            return Optional.empty();
//        }
//
//        // Use Megatag directly when 2 or more tags are visible
//        if (poseEstimate.fiducialIds().length > 1) {
//            return Optional.empty();
//        }
//
//        // Reject if the robot is yawing rapidly (time‑sync unreliable)
//        final double kHighYawLookbackS = 0.3;
//        final double kHighYawVelocityRadS = 5.0;
//
//        if (state.getMaxAbsDriveYawAngularVelocityInRange(
//                        poseEstimate.timestampSeconds() - kHighYawLookbackS,
//                        poseEstimate.timestampSeconds())
//                .orElse(Double.POSITIVE_INFINITY)
//                > kHighYawVelocityRadS) {
//            return Optional.empty();
//        }
//
//        var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
//        if (priorPose.isEmpty()) {
//            return Optional.empty();
//        }
//
//        var maybeFieldToTag =
//                Constants.kAprilTagLayoutReefsOnly.getTagPose(poseEstimate.fiducialIds()[0]);
//        if (maybeFieldToTag.isEmpty()) {
//            return Optional.empty();
//        }
//
//        Pose2d fieldToTag =
//                new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);
//
//        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());
//
//        Pose2d posteriorPose =
//                new Pose2d(
//                        fieldToTag
//                                .getTranslation()
//                                .minus(
//                                        robotToTag
//                                                .getTranslation()
//                                                .rotateBy(priorPose.get().getRotation())),
//                        priorPose.get().getRotation());
//
//        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
//        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
//        double xyStd = Math.max(xStd, yStd);
//
//        return Optional.of(
//                new VisionFieldPoseEstimate(
//                        posteriorPose,
//                        poseEstimate.timestampSeconds(),
//                        VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
//                        poseEstimate.fiducialIds().length));
//    }
}
