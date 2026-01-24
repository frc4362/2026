package com.gemsrobotics.lib.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.gemsrobotics.lib.math.Translation2dPlus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class SwerveUtil {
	private SwerveUtil() {
	}

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
}
