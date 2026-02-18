package com.gemsrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record LimelightPoseEstimateWithVariance(LimelightHelpers.PoseEstimate mt, Matrix<N3, N1> variance) {}
