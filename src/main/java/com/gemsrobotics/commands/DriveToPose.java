package com.gemsrobotics.commands;

import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain m_swerve;
    private final Pose2d m_target;

    public DriveToPose(final CommandSwerveDrivetrain swerve, final Pose2d target) {
        m_swerve = swerve;
        m_target = target;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
