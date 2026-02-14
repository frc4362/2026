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

    public AimAtHub(final CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        m_request = CommandSwerveDrivetrain.makeAimingRequest();
    }

    @Override
    public void execute() {
        final Translation2d swerveLocation = m_swerve.getState().Pose.getTranslation();
        final Rotation2d angleToHub = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint).toTranslation2d().minus(swerveLocation).getAngle();
        m_swerve.setControl(m_request.withTargetDirection(angleToHub));
    }
}
