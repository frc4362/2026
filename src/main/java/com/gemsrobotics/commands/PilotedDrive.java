package com.gemsrobotics.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.lib.math.GeometryUtil;
import com.gemsrobotics.lib.math.Rotation2dPlus;
import com.gemsrobotics.lib.math.Translation2dPlus;
import com.gemsrobotics.lib.swerve.FieldCentricEvasion;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.subsystems.swerve.FieldCentricFacingAngleWithIntakeLimiting;
import com.gemsrobotics.subsystems.swerve.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static com.gemsrobotics.Constants.*;
import static java.lang.Math.*;

public final class PilotedDrive extends Command {
    private static final SwerveRequest.Idle IDLE_REQUEST = new SwerveRequest.Idle();
    private static final boolean DO_PHASE_COMPENSATED_HEADING_GOAL = true;
    public static final Translation2d VELOCITY_ZERO = new Translation2d();

    private final RobotState m_robotState;
    private final CommandSwerveDrivetrain m_swerve;
    private final BooleanSupplier m_isEvading, m_isIntaking, m_isSnaking;
    private final DoubleSupplier m_velocityXSupplier, m_velocityYSupplier, m_rotation;

    private final FieldCentricEvasion m_evasionRequest;
    private final FieldCentricFacingAngleWithIntakeLimiting m_maintainHeadingRequest;
    private final SwerveRequest.Idle m_idleRequest;

    private Optional<Rotation2d> m_maintainHeadingGoal;

    public PilotedDrive(
            final RobotState robotState,
            final CommandSwerveDrivetrain drivetrain,
            final DoubleSupplier velocityX,
            final DoubleSupplier velocityY,
            final DoubleSupplier rotation,
            final BooleanSupplier isEvading,
            final BooleanSupplier isIntaking,
            final BooleanSupplier isSnaking
    ) {
        addRequirements(drivetrain);

        m_robotState = robotState;
        m_swerve = drivetrain;
        m_isEvading = isEvading;
        m_isIntaking = isIntaking;
        m_isSnaking = isSnaking;
        m_velocityXSupplier = velocityX;
        m_velocityYSupplier = velocityY;
        m_rotation = rotation;

        final Translation2d rotationalCenter = new Pose2d().transformBy(FieldConstants.VEHICLE_TO_CENTER).getTranslation();

        m_evasionRequest = new FieldCentricEvasion(SwerveConstants.moduleTranslations, Constants.BUMPER_DEPTH)
                .withDeadband(0.05)
                .withRotationalDeadband(0.025)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withDefaultCenterOfRotation(rotationalCenter)
                .withEvading(false);
        m_maintainHeadingRequest = CommandSwerveDrivetrain.makeAimingRequest();
        m_maintainHeadingRequest.CenterOfRotation = rotationalCenter;
//        m_maintainHeadingRequest.MaxAbsRotationalRate = 1.25 * PI;
        m_idleRequest = new SwerveRequest.Idle();

        m_maintainHeadingGoal = Optional.empty();
    }

    @Override
    public void initialize() {
        m_maintainHeadingGoal = Optional.empty();
    }

    @Override
    public void execute() {
        double stickX = m_velocityXSupplier.getAsDouble();
        double stickY = m_velocityYSupplier.getAsDouble();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            stickX = -stickX;
            stickY = -stickY;
        }
        // DO NOT USE SUPPLIERS BEYOND THIS POINT

        double translationMagnitude = new Translation2dPlus(stickX, stickY).getNorm();
        translationMagnitude = MathUtil.applyDeadband(Math.pow(translationMagnitude, 1.5), 0.025, 1.0); // Scale for low-range movements

        // Maintain drive heading unless turning
        final var deadbandedRotationRate = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.025, 1.0) * MAX_ANGULAR_RATE;
        if (deadbandedRotationRate == 0.0 && translationMagnitude == 0.0) {
            m_swerve.setControl(IDLE_REQUEST);
            return;
        }

        if (translationMagnitude == 0.0) {
            setDriveTurning(VELOCITY_ZERO, deadbandedRotationRate);
            clearHeadingGoal();
        } else {
            Rotation2dPlus translationDirection = new Rotation2dPlus(stickX, stickY);
            // Correct travel direction to nearest 90deg if close to it
            final var nearestPole = translationDirection.getNearestPole();
            if (abs(translationDirection.minus(nearestPole).getDegrees()) < 5) {
                translationDirection = nearestPole;
            }

            final Translation2dPlus targetVelocity = new Translation2dPlus(translationMagnitude * MAX_SPEED, translationDirection);

            if (m_isSnaking.getAsBoolean()) {
                setDrivingFacingAngle(targetVelocity, translationDirection);
                clearHeadingGoal();
            } else if (deadbandedRotationRate == 0.0) {
                // Don't move if not commanding an input
                if (targetVelocity.getNorm() < 0.01) {
                    m_swerve.setControl(m_idleRequest);
                } else {
                    if (m_maintainHeadingGoal.isEmpty()) {
                        m_maintainHeadingGoal = Optional.of(getHeadingGoal());
                    }

                    setDrivingFacingAngle(targetVelocity, m_maintainHeadingGoal.get());
                }
            } else {
                setDriveTurning(targetVelocity, deadbandedRotationRate);
                clearHeadingGoal();
            }
        }
    }

    private void clearHeadingGoal() {
        m_maintainHeadingGoal = Optional.empty();
    }

    // the idea behind this method is that if we don't have a heading goal, but don't want to turn, we should estimate one.
    // it may be more accurate to use our heading before phase lag, ie on the previous robot loop
    // this prevents a brief jump in the system when we swap between modes which do and do not want heading lock
    private Rotation2d getHeadingGoal() {
        if (DO_PHASE_COMPENSATED_HEADING_GOAL) {
            final double currentTime = Timer.getTimestamp();
            final Optional<Pose2d> phaseCompensatedPose = m_robotState.getFieldToVehicle(currentTime - PHASE_LAG_SECONDS);
            if (phaseCompensatedPose.isPresent()) {
                return phaseCompensatedPose.get().getRotation();
            }
        }

        return m_swerve.getState().Pose.getRotation();
    }

    private void setDriveTurning(final Translation2d velocity, final double rotationRate) {
        m_swerve.setControl(m_evasionRequest
                .withIntakeLimiting(m_isIntaking.getAsBoolean())
                .withVelocityX(velocity.getX())
                .withVelocityY(velocity.getY())
                .withRotationalRate(rotationRate)
                .withEvading(m_isEvading.getAsBoolean()));
    }

    private void setDrivingFacingAngle(final Translation2d velocity, final Rotation2d targetAngle) {
        m_swerve.setControl(m_maintainHeadingRequest
                .withIntakeLimiting(m_isIntaking.getAsBoolean())
                .withVelocityX(velocity.getX())
                .withVelocityY(velocity.getY())
                .withTargetDirection(targetAngle));
    }

    // meters per second
    private static final double MAX_ALLOWED_VELOCITY_INTAKING = 2.75;

    private static ChassisSpeeds scaleChassisSpeeds(final ChassisSpeeds desiredVelocity, final double speed) {
        final double currentChassisSpeeds = hypot(desiredVelocity.vxMetersPerSecond, desiredVelocity.vyMetersPerSecond);
        final double scalar = speed / currentChassisSpeeds;
        return new ChassisSpeeds(
                desiredVelocity.vxMetersPerSecond * scalar,
                desiredVelocity.vyMetersPerSecond * scalar,
                desiredVelocity.omegaRadiansPerSecond * scalar);
    }

    public static ChassisSpeeds limitSpeedsForIntaking(final ChassisSpeeds desiredSpeeds, final Rotation2d currentRotation) {
        // we only ever have to worry about slowing down based on the intake corners, as those will be the fastest parts of the robot
        final Transform2d fasterIntakeCorner = desiredSpeeds.omegaRadiansPerSecond >= 0.0 ? INTAKE_CORNER_NE : INTAKE_CORNER_NW;
        // calculate the components of the intake velocity are contributed by the spins
        // we do this out of a method because we want to use them in solving later on
        final double vxSpin = -desiredSpeeds.omegaRadiansPerSecond * (fasterIntakeCorner.getX() * currentRotation.getSin() + fasterIntakeCorner.getY() * currentRotation.getCos());
        final double vySpin = desiredSpeeds.omegaRadiansPerSecond * (fasterIntakeCorner.getX() * currentRotation.getCos() - fasterIntakeCorner.getY() * currentRotation.getSin());
        final double spinInducedSpeed = hypot(vxSpin, vySpin);
        final double vxIntake = desiredSpeeds.vxMetersPerSecond + vxSpin;
        final double vyIntake = desiredSpeeds.vyMetersPerSecond + vySpin;
        final double desiredIntakeSpeed = hypot(vxIntake, vyIntake);
        // this is the final speeds experienced by the faster corner of the intake
        final ChassisSpeeds intakeCornerSpeeds = new ChassisSpeeds(vxIntake, vyIntake, desiredSpeeds.omegaRadiansPerSecond);
        if (spinInducedSpeed > MAX_ALLOWED_VELOCITY_INTAKING || desiredIntakeSpeed > MAX_ALLOWED_VELOCITY_INTAKING) {
            // calculate what we need to lower the intake corner speed to
            // this ONLY works in cases where it is acceptable to change the omega velocity of the robot
            final ChassisSpeeds loweredIntakeSpeeds = scaleChassisSpeeds(intakeCornerSpeeds, MAX_ALLOWED_VELOCITY_INTAKING);
            // and then invert the geometry transformation to the center of the robot to determine what we need to drive the vehicle at
            return GeometryUtil.transformVelocity(loweredIntakeSpeeds, fasterIntakeCorner.inverse(), currentRotation);
        } else {
            return desiredSpeeds;
        }
    }
}
