package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static java.lang.Math.abs;

public final class AimAndBrakeCommand extends Command {
    private static final boolean DO_AIMING_OSCILLATION = false;

    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.FieldCentricFacingAngle m_turnRequest;
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest;
    private final SwerveRequest.Idle m_idleRequest;
    private final Supplier<Optional<Rotation2d>> m_goalSupplier;
    private final DoubleSupplier m_velocityX, m_velocityY;
    private final Timer m_timer;

    private double m_toleranceDegrees;

    public AimAndBrakeCommand(
            final CommandSwerveDrivetrain swerve,
            final Supplier<Optional<Rotation2d>> goalSupplier,
            final DoubleSupplier velocityX,
            final DoubleSupplier velocityY
    ) {
        m_swerve = swerve;
        m_goalSupplier = goalSupplier;
        m_velocityX = velocityX;
        m_velocityY = velocityY;
        m_timer = new Timer();

        m_turnRequest = CommandSwerveDrivetrain.makeAimingRequest();
        m_turnRequest.HeadingController.setTolerance(Math.toRadians(0.5));
        m_turnRequest.HeadingController = new com.ctre.phoenix6.swerve.utility.PhoenixPIDController(24.0, 0.0, 0.0);
        m_idleRequest = new SwerveRequest.Idle();
        m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
        m_brakeRequest.SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;

        // default tolerance
        m_toleranceDegrees = 1.0;

        addRequirements(m_swerve);
    }

//    public AimAndBrakeCommand(final CommandSwerveDrivetrain swerve, final Supplier<Optional<Translation2d>> goalSupplier) {
//        this(swerve, goalSupplier, () -> 0.0, () -> 0.0);
//    }

    public void setTolerance(final Rotation2d newTolerance) {
        m_toleranceDegrees = newTolerance.getDegrees();
    }

    public Optional<Rotation2d> getAngleToGoal() {
        return m_goalSupplier.get();
    }

    public Optional<Rotation2d> getErrorToGoal() {
        return getAngleToGoal().map(angle -> angle.minus(m_swerve.getState().Pose.getRotation()));
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        Optional<Rotation2d> maybeAngleToGoal = getAngleToGoal();
        if (DO_AIMING_OSCILLATION) {
            maybeAngleToGoal = maybeAngleToGoal.map(angle -> {
                final double A = 2.5;
                final double adjustment = (2 * A * (m_timer.get() / (4.0 / 2.0)) % 1) - A;
                return angle.plus(Rotation2d.fromDegrees(adjustment));
            });
        }

        if (maybeAngleToGoal.isPresent()) {
            final Rotation2d angleToHub = maybeAngleToGoal.get();
            m_swerve.setControl(m_turnRequest
                    .withVelocityX(m_velocityX.getAsDouble())
                    .withVelocityY(m_velocityY.getAsDouble())
                    .withTargetDirection(angleToHub));
        } else {
            m_swerve.setControl(m_idleRequest);
        }
    }
}
