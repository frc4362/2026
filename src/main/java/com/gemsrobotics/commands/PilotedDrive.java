package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.Constants;
import com.gemsrobotics.lib.math.Rotation2dPlus;
import com.gemsrobotics.lib.math.Translation2dPlus;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.TunerConstants;
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
    public static final SwerveRequest.Idle IDLE_REQUEST = new SwerveRequest.Idle();
    private final CommandSwerveDrivetrain m_swerve;
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

        m_swerve = drivetrain;
        m_evading = evading;
        m_velocityX = velocityX;
        m_velocityY = velocityY;
        m_rotation = rotation;

        m_evasionRequest = new FieldCentricEvasion(TunerConstants.moduleTranslations, Constants.BUMPER_DEPTH)
                .withDeadband(0.05)
                .withRotationalDeadband(0.025)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withEvading(false);
        m_maintainHeadingRequest = CommandSwerveDrivetrain.makeAimingRequest();
        m_idleRequest = new SwerveRequest.Idle();

        m_maintainHeadingGoal = Optional.empty();
    }

    @Override
    public void execute() {
        // Correct travel direction to nearest 90deg if close to it
        double translationMagnitude = new Translation2dPlus(m_velocityX.getAsDouble(), m_velocityY.getAsDouble()).getNorm();
        translationMagnitude = MathUtil.applyDeadband(Math.pow(translationMagnitude, 1.5), 0.025, 1.0); // Scale for low-range movements

        // Maintain drive heading unless turning
        final var dbRotation = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.025, 1.0) * MAX_ANGULAR_RATE;

        if (dbRotation == 0.0 && translationMagnitude == 0.0) {
            m_swerve.setControl(IDLE_REQUEST);
            return;
        }

        if (translationMagnitude == 0.0) {
            m_swerve.setControl(m_evasionRequest
                    .withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(dbRotation)
                    .withEvading(m_evading.getAsBoolean()));
            m_maintainHeadingGoal = Optional.empty();
        } else {
            Rotation2dPlus translationDirection = new Rotation2dPlus(m_velocityX.getAsDouble(), m_velocityY.getAsDouble());
            final var nearestPole = translationDirection.getNearestPole();
            if (abs(translationDirection.minus(nearestPole).getDegrees()) < 5) {
                translationDirection = nearestPole;
            }

            final Translation2dPlus targetVelocity = new Translation2dPlus(translationMagnitude * MAX_SPEED, translationDirection);

            if (dbRotation == 0.0) {
                // Don't move if not commanding an input
                if (targetVelocity.getNorm() < 0.01) {
                    m_swerve.setControl(m_idleRequest);
                } else {
                    if (m_maintainHeadingGoal.isPresent()) {
                        m_swerve.setControl(m_maintainHeadingRequest
                                .withVelocityX(targetVelocity.getX())
                                .withVelocityY(targetVelocity.getY())
                                .withTargetDirection(m_maintainHeadingGoal.get())); // maintain heading
                    } else {
                        m_swerve.setControl(m_evasionRequest
                                .withVelocityX(targetVelocity.getX())
                                .withVelocityY(targetVelocity.getY())
                                .withEvading(false)
                                .withRotationalRate(0.0));
                    }
                }
            } else {
                m_swerve.setControl(m_evasionRequest
                        .withVelocityX(targetVelocity.getX()) // Drive forward with negative Y (forward)
                        .withVelocityY(targetVelocity.getY()) // Drive left with negative X (left)
                        .withRotationalRate(dbRotation)
                        .withEvading(m_evading.getAsBoolean()));
                m_maintainHeadingGoal = Optional.of(m_swerve.getState().Pose.getRotation());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
