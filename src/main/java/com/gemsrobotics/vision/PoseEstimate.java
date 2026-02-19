package com.gemsrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.struct.MatrixStruct;
import edu.wpi.first.util.struct.Struct;

import java.nio.ByteBuffer;

public record PoseEstimate(
        double timestampSeconds,
        Pose2d fieldToVehicle,
        Matrix<N3, N1> variance,
        int tagCount
) {
	// TODO make this a struct...
	private static final Struct<Matrix<N3, N1>> VARIANCE_MAT_STRUCT = Matrix.getStruct(Nat.N3(), Nat.N1());

	public static class PoseEstimateStruct implements Struct<PoseEstimate> {
		@Override
		public Class<PoseEstimate> getTypeClass() {
			return PoseEstimate.class;
		}

		@Override
		public String getTypeName() {
			return "PoseEstimate";
		}

		@Override
		public int getSize() {
			return Struct.kSizeDouble + Pose2d.struct.getSize() + VARIANCE_MAT_STRUCT.getSize() + Integer.BYTES;
		}

		@Override
		public String getSchema() {
			return "double timestampSeconds; Pose2d fieldToVehicle; " + VARIANCE_MAT_STRUCT.getTypeName() + "; int tagCount";
		}

		@Override
		public PoseEstimate unpack(final ByteBuffer byteBuffer) {
			final double timestampSeconds = byteBuffer.getDouble();
			final Pose2d pose = Pose2d.struct.unpack(byteBuffer);
			final Matrix<N3, N1> variance = VARIANCE_MAT_STRUCT.unpack(byteBuffer);
			final int tagCount = byteBuffer.getInt();
			return new PoseEstimate(timestampSeconds, pose, variance, tagCount);
		}

		@Override
		public void pack(final ByteBuffer byteBuffer, final PoseEstimate poseEstimate) {
			byteBuffer.putDouble(poseEstimate.timestampSeconds);
			Pose2d.struct.pack(byteBuffer, poseEstimate.fieldToVehicle);
			VARIANCE_MAT_STRUCT.pack(byteBuffer, poseEstimate.variance);
			byteBuffer.putInt(poseEstimate.tagCount);
		}

		@Override
		public boolean isImmutable() {
			return true;
		}
	}

	public static PoseEstimateStruct struct =  new PoseEstimateStruct();
}
