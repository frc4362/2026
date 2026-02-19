package com.gemsrobotics.vision;

import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.*;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public final class Vision {
    private final RobotState m_robotState;
    private final Limelight4 m_cameraLauncher, m_cameraClimber;
    private final List<Limelight4> m_cameras;
    private final Supplier<Limelight4.Inputs> m_inputSupplier;

    private final NetworkTable m_table, m_camerasTable;
    private final Map<String, VisionProcessingResultsLogger> m_cameraLoggers;

    private boolean m_hasBeenEnabled;

    public Vision(final RobotState robotState, final Supplier<Limelight4.Inputs> inputSupplier) {
        m_robotState = robotState;
        m_inputSupplier = inputSupplier;

        m_table = NetworkTableInstance.getDefault().getTable("vision");
        m_camerasTable = m_table.getSubTable("cameras");

        m_cameraLauncher = new Limelight4(Constants.Vision.LIMELIGHT_LAUNCHER_NAME, Constants.Vision.LIMELIGHT_LAUNCHER_TRANSFORM);
        m_cameraClimber = new Limelight4(Constants.Vision.LIMELIGHT_CLIMBER_NAME, Constants.Vision.LIMELIGHT_CLIMBER_TRANSFORM);

        m_cameras =  Arrays.asList(m_cameraLauncher, m_cameraClimber);
        m_cameraLoggers = new HashMap<>(m_cameras.size());
        m_cameras.forEach(camera -> {
            final String name = camera.getName();
            m_cameraLoggers.put(name, new VisionProcessingResultsLogger(m_camerasTable, name));
        });

        m_hasBeenEnabled = false;
    }

    public void configureCamerasEnabled() {
        m_cameras.forEach(Limelight4::configureEnabled);
        m_hasBeenEnabled = true;
    }

    public void configureCamerasDisabled() {
        m_cameras.forEach(Limelight4::configureDisabled);
    }

    public void update() {
        var inputs = m_inputSupplier.get();
        final List<PoseEstimate> estimates = m_cameras.stream()
                .flatMap(camera -> camera.update(inputs).stream())
                .map(this::processCameraOutputs)
                .flatMap(Optional::stream)
                .toList();

        // TODO give to the robot state either the one pose estimate available, or the fused pose estimate
        // make sure to log if its accepted

        PoseEstimate acceptedEstimate = null;
        if (estimates.size() == 1) {
            acceptedEstimate = estimates.get(0);
        } else if (estimates.size() > 1) {
            final Optional<PoseEstimate> fusedEstimate = fusePoseEstimates(estimates);
            if (fusedEstimate.isPresent()) {
                acceptedEstimate = fusedEstimate.get();
            }
        }

        if (acceptedEstimate != null) {
            m_robotState.updatePoseEstimate(acceptedEstimate);
            // TODO log the pose estimate struct
        }
    }

    private Optional<PoseEstimate> processCameraOutputs(final Limelight4.Outputs outputs) {
        if (!outputs.hasTags()) {
            return Optional.empty();
        }

        final Optional<PoseEstimate> megatagEstimate = outputs.getBestPoseEstimate()
                .filter(b -> b.estimate().tagCount > 1)
                .flatMap(this::processLimelightPoseEstimate);

        final LimelightPoseEstimateWithVariance mt1estimate = outputs.mt1();
        final Optional<PoseEstimate> gyroFusedEstimate = processGyroFusedPoseEstimate(mt1estimate);
        final Optional<PoseEstimate> selectedEstimate = megatagEstimate.or(() -> gyroFusedEstimate);

        selectedEstimate.ifPresent(estimate -> {
            final var logger = m_cameraLoggers.get(outputs.cameraName());
            if (!Objects.isNull(logger)) {
                logger.log(estimate.timestampSeconds(),
                        megatagEstimate.map(PoseEstimate::fieldToVehicle),
                        gyroFusedEstimate.map(PoseEstimate::fieldToVehicle));
            }
        });

        return selectedEstimate;
    }

    // we are assuming that all the pose estimates are independent, and do not share a source of error ie. field layout
    private Optional<PoseEstimate> fusePoseEstimates(PoseEstimate a, PoseEstimate b) {
        if (b.timestampSeconds() < a.timestampSeconds()) {
            var temp = a;
            a = b;
            b = temp;
        }

        final Optional<Pose2d> maybeCaptureA = m_robotState.getFieldToVehicle(a.timestampSeconds());
        final Optional<Pose2d> maybeCaptureB = m_robotState.getFieldToVehicle(b.timestampSeconds());

        if (maybeCaptureA.isEmpty() || maybeCaptureB.isEmpty()) {
            return Optional.empty();
        }

        // apply the difference in when we took the two measurements to the earlier measurement
        // effectively scrubbing us forward in time
        final Transform2d aTb = maybeCaptureA.get().minus(maybeCaptureB.get());
        final Pose2d poseA = a.fieldToVehicle().transformBy(aTb);
        final Pose2d poseB = b.fieldToVehicle();

        // square each element of the variance
        final Matrix<N3, N1> varianceA = a.variance().elementTimes(a.variance());
        final Matrix<N3, N1> varianceB = b.variance().elementTimes(b.variance());

        // compare the headings of the two readings and perform a weighted average of them
        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < Constants.Vision.HIGH_VARIANCE && varianceB.get(2, 0) < Constants.Vision.HIGH_VARIANCE) {
            fusedHeading = new Rotation2d(
                poseA.getRotation().getCos() / varianceA.get(2, 0) + poseB.getRotation().getCos() / varianceB.get(2, 0),
                poseA.getRotation().getSin() / varianceA.get(2, 0) + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        final double weightAx = 1.0 / varianceA.get(0, 0);
        final double weightAy = 1.0 / varianceA.get(1, 0);
        final double weightBx = 1.0 / varianceB.get(0, 0);
        final double weightBy = 1.0 / varianceB.get(1, 0);

        final Translation2d weightedTranslation = new Translation2d(
                (poseA.getTranslation().getX() * weightAx + poseB.getTranslation().getX() * weightBx) / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy + poseB.getTranslation().getY() * weightBy) / (weightAy + weightBy));

        final var fusedPose = new Pose2d(weightedTranslation, fusedHeading);

        final Matrix<N3, N1> fusedVariance = VecBuilder.fill(
                Math.sqrt(1.0 / (weightAx + weightBx)),
                Math.sqrt(1.0 / (weightAy + weightBy)),
                Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

        // the time of them should be equal
        return Optional.of(new PoseEstimate(
                b.timestampSeconds(),
                fusedPose,
                fusedVariance,
                a.tagCount() + b.tagCount()));
    }

    // we are assuming that all the pose estimates are independent, and do not share a source of error ie. field layout
    private Optional<PoseEstimate> fusePoseEstimates(List<PoseEstimate> visionEstimates) {
        // ensure the estimates are all in order
        visionEstimates.sort(Comparator.comparing(PoseEstimate::timestampSeconds));

        // filter the estimates, make sure we have a capture location for each.
        // use a LinkedList for fast insertion at the front
        final List<Pose2d> capturePoses = new LinkedList<>();
        for (int i = visionEstimates.size() - 1; i >= 0; i--) {
            final Optional<Pose2d> maybeCapturePose = m_robotState.getFieldToVehicle(visionEstimates.get(i).timestampSeconds());
            if (maybeCapturePose.isPresent()) {
                // we are iterating over this backwards, so we should build the list backwards.
                // ie, always insert at the start
                capturePoses.add(0, maybeCapturePose.get());
            } else {
                visionEstimates.remove(i);
            }
        }

        // if we have less estimates than we can reasonably fuse, just return the estimate (or nothing)
        if (visionEstimates.size() < 2) {
            return visionEstimates.stream().findFirst();
        }

        final PoseEstimate mostRecentPoseEstimate = visionEstimates.get(visionEstimates.size() - 1);
        final Pose2d mostRecentCapturePose = capturePoses.get(capturePoses.size() - 1);

        // apply the difference in when we took the measurements to the earlier measurements
        // effectively scrubbing us forward in time for everything but the most recent one
        final List<Pose2d> synchronizedPoses = new ArrayList<>(capturePoses.size());
        // don't need to fast-forward the most recent element, so it's size() - 2
        for (int i = 0; i <= capturePoses.size() - 2; i++) {
            // apply the measurement-pose difference to the earlier estimate
            // vision transform = later pose - earlier pose capture location
            // synchronized estimate = vision estimate + vision transform
            final Transform2d nTr = mostRecentCapturePose.minus(capturePoses.get(i));
            synchronizedPoses.add(visionEstimates.get(i).fieldToVehicle().transformBy(nTr));
        }
        // add the last pose which we did not need to fast-forward
        synchronizedPoses.add(mostRecentCapturePose);

        // square each element of the variance
        final List<Matrix<N3, N1>> variances = visionEstimates.stream().map(
                estimate -> estimate.variance().elementTimes(estimate.variance())).toList();

        // default to the heading of the most recent reading if we can't fuse them
        Rotation2d fusedHeading;
        if (variances.stream().allMatch(variance -> variance.get(2, 0) < Constants.Vision.HIGH_VARIANCE)) {
            // perform the averaging in vector-space, so that wrapping isn't an issue
            // note that we do not need to divide by the weights at the end as they are internally normalized in Rotation2d
            double cosSum = 0.0;
            double sinSum = 0.0;
            for (int i = 0; i < synchronizedPoses.size() - 1; i++) {
                cosSum += synchronizedPoses.get(i).getRotation().getCos() / variances.get(i).get(2, 0);
                sinSum += synchronizedPoses.get(i).getRotation().getSin() / variances.get(i).get(2, 0);
            }

            fusedHeading = new Rotation2d(cosSum, sinSum);
        } else {
            fusedHeading = mostRecentCapturePose.getRotation();
        }

        // variables to perform a Bayesian fusion of the translations
        double totalX = 0.0;
        double totalY = 0.0;
        double precisionX = 0.0;
        double precisionY = 0.0;
        // variables to calculate the precision of our rotation estimate
        double precisionTheta = 0.0;
        for (int i = 0; i <= synchronizedPoses.size() - 1; i++) {
            final double newPrecisionX = 1.0 / variances.get(i).get(0, 0);
            final double newPrecisionY = 1.0 / variances.get(i).get(1, 0);

            totalX += synchronizedPoses.get(i).getTranslation().getX() * newPrecisionX;
            totalY += synchronizedPoses.get(i).getTranslation().getY() * newPrecisionY;

            precisionX += newPrecisionX;
            precisionY += newPrecisionY;

            // sum the precision of our theta
            precisionTheta += 1.0 / variances.get(i).get(2, 0);
        }

        final Translation2d weightedTranslation = new Translation2d(totalX / precisionX, totalY / precisionY);
        final var fusedPose = new Pose2d(weightedTranslation, fusedHeading);

        final Matrix<N3, N1> fusedVariance = VecBuilder.fill(
                Math.sqrt(1.0 / precisionX),
                Math.sqrt(1.0 / precisionY),
                Math.sqrt(1.0 / precisionTheta));

        final int totalTagCount = visionEstimates.stream().mapToInt(PoseEstimate::tagCount).sum();
        
        return Optional.of(new PoseEstimate(
                mostRecentPoseEstimate.timestampSeconds(),
                fusedPose,
                fusedVariance,
                totalTagCount));
    }

    private Optional<PoseEstimate> processLimelightPoseEstimate(final LimelightPoseEstimateWithVariance poseEstimateWithVariance) {
        final var poseEstimate = poseEstimateWithVariance.estimate();
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
                VecBuilder.fill(xyVar, xyVar, variance.get(0, 2)).times(varianceScalar),
                poseEstimate.tagCount));
    }

    private Optional<PoseEstimate> processGyroFusedPoseEstimate(final LimelightPoseEstimateWithVariance poseEstimateWithVariance) {
        final var poseEstimate = poseEstimateWithVariance.estimate();
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
                VecBuilder.fill(xyVar, xyVar, Constants.Vision.HIGH_VARIANCE),
                poseEstimate.rawFiducials.length));
    }
}
