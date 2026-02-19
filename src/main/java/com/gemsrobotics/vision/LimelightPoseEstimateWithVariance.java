package com.gemsrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record LimelightPoseEstimateWithVariance(LimelightHelpers.PoseEstimate estimate, Matrix<N3, N1> variance) {
    public boolean isValid() {
        return estimate.tagCount > 0;
    }

    public boolean isInvalid() {
        return estimate.tagCount == 0;
    }
}
