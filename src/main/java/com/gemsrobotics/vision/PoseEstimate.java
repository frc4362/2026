package com.gemsrobotics.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

public record PoseEstimate(double timestampSeconds, Pose2d fieldToVehicle, Matrix<N3, N1> variance, int tagCount,
						   boolean isFusedEstimate) implements StructSerializable {
	public static final PoseEstimate NULL = new PoseEstimate(Double.NaN, Pose2d.kZero, VecBuilder.fill(0.0, 0.0, 0.0), 0);

	public PoseEstimate(
			final double timestampSeconds,
			final Pose2d fieldToVehicle,
			final Matrix<N3, N1> variance,
			final int tagCount
	) {
		this(timestampSeconds, fieldToVehicle, variance, tagCount, false);
	}


	@Override
	public String toString() {
		return "PoseEstimate[" +
				"timestampSeconds=" + timestampSeconds + ", " +
				"fieldToVehicle=" + fieldToVehicle + ", " +
				"variance=" + variance + ", " +
				"tagCount=" + tagCount + ", " +
				"isFusedEstimate=" + isFusedEstimate + ']';
	}

	public static final PoseEstimateStruct struct = new PoseEstimateStruct();
	private static Struct<Matrix<N3, N1>> VARIANCE_STRUCT = Matrix.getStruct(Nat.N3(), Nat.N1());

	public static class PoseEstimateStruct implements Struct<PoseEstimate> {
		@Override
		public Class<PoseEstimate> getTypeClass() {
			return PoseEstimate.class;
		}

		@Override
		public String getTypeName() {
			return "record:PoseEstimate";
		}

		@Override
		public int getSize() {
			return Struct.kSizeDouble + Pose2d.struct.getSize() + VARIANCE_STRUCT.getSize() + Integer.BYTES + Struct.kSizeBool;
		}

		@Override
		public String getSchema() {
			return "double timestampSeconds; Pose2d fieldToVehicle; " + VARIANCE_STRUCT.getTypeName() + " variance; int tagCount; boolean isFusedEstimate";
		}

		@Override
		public PoseEstimate unpack(final ByteBuffer byteBuffer) {
			final double timestampSeconds = byteBuffer.getDouble();
			final Pose2d pose = Pose2d.struct.unpack(byteBuffer);
			final Matrix<N3, N1> variance = VARIANCE_STRUCT.unpack(byteBuffer);
			final int tagCount = byteBuffer.getInt();
			final boolean isFusedEstimate = (byteBuffer.get() == (byte) 1);
			return new PoseEstimate(timestampSeconds, pose, variance, tagCount, isFusedEstimate);
		}

		@Override
		public void pack(final ByteBuffer byteBuffer, final PoseEstimate poseEstimate) {
			byteBuffer.putDouble(poseEstimate.timestampSeconds);
			Pose2d.struct.pack(byteBuffer, poseEstimate.fieldToVehicle);
			VARIANCE_STRUCT.pack(byteBuffer, poseEstimate.variance);
			byteBuffer.putInt(poseEstimate.tagCount);
			byteBuffer.put((byte) (poseEstimate.isFusedEstimate ? 1 : 0));
		}

		@Override
		public boolean isImmutable() {
			return true;
		}
	}
}
