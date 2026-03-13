package com.gemsrobotics.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.choreo.ChoreoTraj;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class Autos {

    private final RobotContainer m_robot;
    private final Superstructure m_superstructure;
    private final CommandSwerveDrivetrain m_swerve;
    private final AutoFactory m_autoFactory;
    private final AutoChooser m_autoChooser;

    public Autos(RobotContainer robot) {
        m_robot = robot;
        m_autoFactory = robot.getSwerve().createAutoFactory();
        m_autoFactory
                .bind("Intake", m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.INTAKING))
                .bind("StopIntake", m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE));

        m_superstructure = m_robot.getSuperstructure();
        m_swerve = m_robot.getSwerve();

        m_autoChooser = new AutoChooser();
        m_autoChooser.addRoutine("Left Auto", this::leftShoot);
        m_autoChooser.addRoutine("Left Auto Skip Bump", this::leftSkipBump);
        m_autoChooser.addRoutine("Right Auto", this::rightShoot);
        m_autoChooser.addRoutine("Right Auto Skip Bump", this::rightSkipBump);
        m_autoChooser.addRoutine("Home Auto", this::homeAuto);
    }

    public AutoRoutine leftShoot() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Left Auto");
        final AutoTrajectory driveRightPath = ChoreoTraj.LeftBasicShootAuto.asAutoTraj(routine);

        routine.active().onTrue(Commands.parallel(
                driveRightPath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));

        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(10.0));

        return routine;
    }

    public AutoRoutine rightShoot() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Right Auto");
        final AutoTrajectory driveRightPath = ChoreoTraj.RightBasicShootAuto.asAutoTraj(routine);

        routine.active().onTrue(Commands.parallel(
                driveRightPath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));

        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(10.0));

        return routine;
    }

    public AutoRoutine leftSkipBump() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Left Skip Bump Auto");
        final AutoTrajectory drivePath = ChoreoTraj.LeftSkipBump.asAutoTraj(routine);
        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.75, 5.5)));

        final AutoTrajectory drivePath2 = ChoreoTraj.LeftSkipBump2.asAutoTraj(routine);

        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
                .andThen(() -> m_swerve.resetTranslation(startingTranslation)));
        routine.active().onTrue(Commands.parallel(
                drivePath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
                .withTimeout(9.0)
                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
                .andThen(drivePath2.cmd()));
        drivePath2.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
                .withTimeout(9.0)
                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoRoutine rightSkipBump() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Right Skip Bump Auto");
        final AutoTrajectory drivePath = ChoreoTraj.RightSkipBump.asAutoTraj(routine);
        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.75, 2.42)));

        final AutoTrajectory drivePath2 = ChoreoTraj.RightSkipBump2.asAutoTraj(routine);

        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
                .andThen(() -> m_swerve.resetTranslation(startingTranslation)));
        routine.active().onTrue(Commands.parallel(
                drivePath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
                .withTimeout(9.0)
                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
                .andThen(drivePath2.cmd()));
        drivePath2.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
                .withTimeout(9.0)
                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoRoutine homeAuto() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Home Auto");
        final AutoTrajectory driveRightPath = ChoreoTraj.HomeAuto.asAutoTraj(routine);

        routine.active().onTrue(Commands.parallel(
                driveRightPath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));

        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()));
        return routine;
    }

    public AutoChooser getAutoChooser() {
        return m_autoChooser;
    }
}
