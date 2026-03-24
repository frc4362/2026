package com.gemsrobotics.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.gemsrobotics.commands.PilotedDrive;
import com.gemsrobotics.lib.math.Translation2dPlus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

public class FieldCentricEvasion implements SwerveRequest {
    public boolean Evading = false;
    public boolean DoIntakeLimiting = false;

    public FieldCentricEvasion withIntakeLimiting(final boolean doIntakeLimiting) {
        DoIntakeLimiting = doIntakeLimiting;
        return this;
    }

    /**
     * The velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;
    /**
     * The allowable deadband of the request.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request.
     */
    public double RotationalDeadband = 0;

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;

    /**
     * Used to apply specific states and requests to motors.
     */
    public SwerveModule.ModuleRequest ModuleRequest = new SwerveModule.ModuleRequest();

    public FieldCentricEvasion withEvading(final boolean b) {
        this.Evading = b;
        return this;
    }

    /**
     * Sets the velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public FieldCentricEvasion withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public FieldCentricEvasion withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     *
     * @param rotationalRate Angular rate to rotate at, in radians per second
     * @return this request
     */
    public FieldCentricEvasion withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public FieldCentricEvasion withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public FieldCentricEvasion withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public FieldCentricEvasion withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public FieldCentricEvasion withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }

    private static final Translation2d ORIGIN = new Translation2d();

    private final Translation2d[] m_wheelPositions;
    private final double m_bumperThickness;

    public FieldCentricEvasion(final Translation2d[] moduleTranslations, final Distance bumperThickness) {
        m_wheelPositions = moduleTranslations;
        m_bumperThickness = bumperThickness.in(Meters);
    }

    private Translation2d getEvadingCenter(final Rotation2d direction, final Rotation2d currentHeading, final double rotation) {
        final var here = new Translation2dPlus(1.0, direction.minus(currentHeading));
        // final Translation2dPlus here = null;
        var cwCenter = m_wheelPositions[0];
        var ccwCenter = m_wheelPositions[m_wheelPositions.length - 1];

        for (int i = 0; i < m_wheelPositions.length - 1; i++) {
            final var cw = m_wheelPositions[i];
            final var ccw = m_wheelPositions[i + 1];

            if (here.isWithinAngle(cw, ccw)) {
                cwCenter = ccw;
                ccwCenter = cw;
            }
        }

        // if we're not turning, don't change the center of rotation
        if (rotation == 0.0) {
            return ORIGIN;
        }

        final Translation2d newCenter;

        // if clockwise
        if (Math.signum(rotation) == 1.0) {
            newCenter = cwCenter;
        } else if (Math.signum(rotation) == -1.0) {
            newCenter = ccwCenter;
        } else { // just in case
            return ORIGIN;
        }

        return new Translation2d(newCenter.getNorm() + 2 * m_bumperThickness, newCenter.getAngle());
    }

    private static final boolean DO_OPERATOR_PERSPECTIVE = false;

    public StatusCode apply(SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        final Translation2d v;
        if (DO_OPERATOR_PERSPECTIVE) {
            v = new Translation2d(VelocityX, VelocityY).rotateBy(parameters.operatorForwardDirection);
        } else {
            v = new Translation2d(VelocityX, VelocityY);
        }

        double toApplyX = v.getX();
        double toApplyY = v.getY();
        double toApplyOmega = RotationalRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
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

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(CorrectVelocityX, CorrectVelocityY, CorrectVelocityOmega,
                parameters.currentPose.getRotation()), parameters.updatePeriod);

        final Translation2d centerOfRotation;
        if (Evading) {
            centerOfRotation = getEvadingCenter(new Rotation2d(toApplyX, toApplyY), parameters.currentPose.getRotation(), toApplyOmega);
        } else {
            centerOfRotation = ORIGIN;
        }

        var states = parameters.kinematics.toSwerveModuleStates(speeds, centerOfRotation);
        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(ModuleRequest.withState(states[i]));
        }

        return StatusCode.OK;
    }
}