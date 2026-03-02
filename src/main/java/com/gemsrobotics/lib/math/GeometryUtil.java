package com.gemsrobotics.lib.math;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeometryUtil {
	public static ChassisSpeeds fieldSpeeds(final SwerveDrivetrain.SwerveDriveState state) {
		final var newSpeeds = new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)
				.rotateBy(state.Pose.getRotation());
		return new ChassisSpeeds(newSpeeds.getX(), newSpeeds.getY(), state.Speeds.omegaRadiansPerSecond);
	}

	public static double velocityTowardsPoint(final SwerveDrivetrain.SwerveDriveState state, final Translation2d point) {
		final var speeds = fieldSpeeds(state);
		return new Translation2dPlus(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).scal(point.minus(state.Pose.getTranslation()));
	}

	public static double velocityScalar(final SwerveDrivetrain.SwerveDriveState state) {
		return Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
	}

	public static ChassisSpeeds transformVelocity(
			final ChassisSpeeds velocity,
			final Transform2d transform,
			final Rotation2d currentRotation
	) {
		return new ChassisSpeeds(
				velocity.vxMetersPerSecond + velocity.omegaRadiansPerSecond * (transform.getY() * currentRotation.getCos() - transform.getX() * currentRotation.getSin()),
				velocity.vyMetersPerSecond + velocity.omegaRadiansPerSecond * (transform.getX() * currentRotation.getCos() - transform.getY() * currentRotation.getSin()),
				velocity.omegaRadiansPerSecond);
	}
}
