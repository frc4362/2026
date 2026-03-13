package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.Supplier;

import static java.lang.Math.abs;

public final class AimAndBrakeCommand extends Command {
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.FieldCentricFacingAngle m_turnRequest;
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest;
    private final SwerveRequest.Idle m_idleRequest;
    private final Supplier<Optional<Translation2d>> m_goalSupplier;

    private double m_toleranceDegrees;

    public AimAndBrakeCommand(final CommandSwerveDrivetrain swerve, final Supplier<Optional<Translation2d>> goalSupplier) {
        m_swerve = swerve;
        m_goalSupplier = goalSupplier;

        m_turnRequest = CommandSwerveDrivetrain.makeAimingRequest();
        m_idleRequest = new SwerveRequest.Idle();
        m_brakeRequest = new  SwerveRequest.SwerveDriveBrake();
        m_brakeRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;

        // default tolerance
        m_toleranceDegrees = 1.0;

        addRequirements(m_swerve);
    }

    public void setTolerance(final Rotation2d newTolerance) {
        m_toleranceDegrees = newTolerance.getDegrees();
    }

    public Optional<Rotation2d> getAngleToGoal() {
        return m_goalSupplier.get().map(goal ->
            goal.minus(m_swerve.getState().Pose.getTranslation()).getAngle());
    }

    public Optional<Rotation2d> getErrorToGoal() {
        return getAngleToGoal().map(angle -> angle.minus(m_swerve.getState().Pose.getRotation()));
    }

    @Override
    public void execute() {
        final Optional<Rotation2d> maybeAngleToGoal = getAngleToGoal();
        if (maybeAngleToGoal.isPresent()) {
            final Rotation2d angleToHub = maybeAngleToGoal.get();
            // TODO
//            if (m_toleranceDegrees == 0.0 || abs(angleToHub.getDegrees()) > m_toleranceDegrees) {
                m_swerve.setControl(m_turnRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(angleToHub));
//            } else {
//                m_swerve.setControl(m_brakeRequest);
//            }
        } else {
            m_swerve.setControl(m_idleRequest);
        }
    }
}
