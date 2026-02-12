package com.gemsrobotics.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.Constants;
import com.gemsrobotics.lib.math.Rotation2dPlus;
import com.gemsrobotics.lib.math.Translation2dPlus;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.gemsrobotics.Constants.MAX_ANGULAR_RATE;
import static com.gemsrobotics.Constants.MAX_SPEED;
import static java.lang.Math.abs;

public class PilotedDrive extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final BooleanSupplier m_evading;
    private final DoubleSupplier m_velocityX, m_velocityY, m_rotation;

    private final FieldCentricEvasion m_evasionRequest;
    private final SwerveRequest.FieldCentricFacingAngle m_maintainHeadingRequest;
    private final SwerveRequest.Idle m_idleRequest;

    private Optional<Rotation2d> m_maintainHeadingGoal;

    public PilotedDrive(
            final CommandSwerveDrivetrain drivetrain,
            final BooleanSupplier evading,
            final DoubleSupplier velocityX,
            final DoubleSupplier velocityY,
            final DoubleSupplier rotation
    ) {
        addRequirements(drivetrain);

        m_drivetrain = drivetrain;
        m_evading = evading;
        m_velocityX = velocityX;
        m_velocityY = velocityY;
        m_rotation = rotation;

        m_evasionRequest = new FieldCentricEvasion(TunerConstants.moduleTranslations, Constants.BUMPER_DEPTH)
                .withDeadband(0.05)
                .withRotationalDeadband(0.1)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withEvading(false);
        m_maintainHeadingRequest = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(0.05)
                .withRotationalDeadband(0.1)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
        m_idleRequest = new SwerveRequest.Idle();

        m_maintainHeadingGoal = Optional.empty();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Correct travel direction to nearest 90deg if close to it
        double stickMagnitude = new Translation2dPlus(m_velocityX.getAsDouble(), m_velocityY.getAsDouble()).getNorm();
        stickMagnitude = MathUtil.applyDeadband(Math.pow(stickMagnitude, 1.5), 0.025, 1.0); // Scale for low-range movements

        Rotation2dPlus stickDirection = new Rotation2dPlus(m_velocityX.getAsDouble(), m_velocityY.getAsDouble());
        final var nearestPole = stickDirection.getNearestPole();
        if (abs(stickDirection.minus(nearestPole).getDegrees()) < 5) {
            stickDirection = nearestPole;
        }

        final Translation2dPlus targetVelocity = new Translation2dPlus(stickMagnitude * MAX_SPEED, stickDirection);

        // Maintain drive heading unless turning
        final var dbRotation = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.025, 1.0) * MAX_ANGULAR_RATE;

        if (dbRotation == 0.0) {
            // Don't move if not commanding an input
            if (targetVelocity.getNorm() < 0.01) {
                m_drivetrain.setControl(m_idleRequest);
            } else {
                m_drivetrain.setControl(m_maintainHeadingRequest
                        .withVelocityX(targetVelocity.getX())
                        .withVelocityY(targetVelocity.getY())
                        .withTargetDirection(m_maintainHeadingGoal.orElse(Rotation2d.kZero))); // maintain heading
            }
        } else {
            m_drivetrain.setControl(m_evasionRequest
                    .withVelocityX(targetVelocity.getX()) // Drive forward with negative Y (forward)
                    .withVelocityY(targetVelocity.getY()) // Drive left with negative X (left)
                    .withRotationalRate(dbRotation)
                    .withEvading(m_evading.getAsBoolean()));
            m_maintainHeadingGoal = Optional.of(m_drivetrain.getState().Pose.getRotation());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
