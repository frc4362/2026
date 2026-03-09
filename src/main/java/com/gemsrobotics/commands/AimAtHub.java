package com.gemsrobotics.commands;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class AimAtHub extends Command {
    private final CommandSwerveDrivetrain m_swerve;
    private final SwerveRequest.FieldCentricFacingAngle m_request;
    private Rotation2d m_angleToHub;

    public AimAtHub(final CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_request = CommandSwerveDrivetrain.makeAimingRequest();
        addRequirements(m_swerve);
    }

    private Rotation2d getAngleToHub() {
        final Translation2d swerveLocation = m_swerve.getState().Pose.getTranslation();
        return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d().minus(swerveLocation).getAngle();
    }

    @Override
    public void initialize() {
        m_angleToHub = getAngleToHub();
    }

    @Override
    public void execute() {
        m_swerve.setControl(m_request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withTargetDirection(getAngleToHub()));
    }

    @Override
    public boolean isFinished() {
        return getAngleToHub().getDegrees() < 1.0;
    }
}
