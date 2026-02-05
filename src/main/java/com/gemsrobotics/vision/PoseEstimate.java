package com.gemsrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record PoseEstimate(
        double timestampSeconds,
        Pose2d robotPose,
        Matrix<N3, N1> variance,
        int numTags
) {
}
