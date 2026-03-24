package com.gemsrobotics.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.commands.PilotedDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class FieldCentricFacingAngleWithIntakeLimiting extends SwerveRequest.FieldCentricFacingAngle {
    public boolean DoIntakeLimiting;
    private final FieldCentric m_myFieldCentric;

    public FieldCentricFacingAngleWithIntakeLimiting() {
        super();
        DoIntakeLimiting = false;
        m_myFieldCentric = new FieldCentric();
    }

    public FieldCentricFacingAngleWithIntakeLimiting withIntakeLimiting(final boolean doIntakeLimiting) {
        DoIntakeLimiting = doIntakeLimiting;
        return this;
    }

    @Override
    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Rotation2d angleToFace = TargetDirection;
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /* If we're operator perspective, rotate the direction we want to face by the angle */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }

        double toApplyOmega = TargetRateFeedforward +
                HeadingController.calculate(
                        parameters.currentPose.getRotation().getRadians(),
                        angleToFace.getRadians(),
                        parameters.timestamp
                );
        if (MaxAbsRotationalRate > 0.0) {
            if (toApplyOmega > MaxAbsRotationalRate) {
                toApplyOmega = MaxAbsRotationalRate;
            } else if (toApplyOmega < -MaxAbsRotationalRate) {
                toApplyOmega = -MaxAbsRotationalRate;
            }
        }

        final double CorrectVelocityX;
        final double CorrectVelocityY;
        final double CorrectVelocityOmega;
        if (DoIntakeLimiting) {
            final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(VelocityX, VelocityY, toApplyOmega);
            final ChassisSpeeds limitedSpeeds = PilotedDrive.limitSpeedsForIntaking(desiredSpeeds, parameters.currentPose.getRotation());
            CorrectVelocityX = limitedSpeeds.vxMetersPerSecond;
            CorrectVelocityY = limitedSpeeds.vyMetersPerSecond;
            CorrectVelocityOmega = limitedSpeeds.omegaRadiansPerSecond;
        } else {
            CorrectVelocityX = VelocityX;
            CorrectVelocityY = VelocityY;
            CorrectVelocityOmega = toApplyOmega;
        }

        return m_myFieldCentric
                .withVelocityX(CorrectVelocityX)
                .withVelocityY(CorrectVelocityY)
                .withRotationalRate(CorrectVelocityOmega)
                .withDeadband(Deadband)
                .withRotationalDeadband(RotationalDeadband)
                .withCenterOfRotation(CenterOfRotation)
                .withDriveRequestType(DriveRequestType)
                .withSteerRequestType(SteerRequestType)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withForwardPerspective(ForwardPerspective)
                .apply(parameters, modulesToApply);
    }
}
