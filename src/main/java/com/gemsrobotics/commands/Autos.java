package com.gemsrobotics.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;

import static java.lang.Math.abs;

public final class Autos {
    private final RobotContainer m_robot;
    private final RobotState m_robotState;
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

        m_robotState = m_robot.getRobotState();
        m_superstructure = m_robot.getSuperstructure();
        m_swerve = m_robot.getSwerve();

        m_autoChooser = new AutoChooser();
        m_autoChooser.addRoutine("Bline Right Auto", () -> blineAuto_RightSkipBump(false));
        m_autoChooser.addRoutine("Bline Left Auto", this::blineAuto_Left);
//        m_autoChooser.addRoutine("Left Auto", this::leftShoot);
//        m_autoChooser.addRoutine("Left Auto Hot", this::leftHotAuto);
//        m_autoChooser.addRoutine("Left Auto Skip Bump", this::leftSkipBump);
////        m_autoChooser.addRoutine("Right Auto", this::rightShoot);
//        m_autoChooser.addRoutine("Right Auto Skip Bump", this::rightSkipBump);
//        m_autoChooser.addRoutine("Right Auto Hot", this::rightHotAuto);
//        m_autoChooser.addRoutine("Home Auto", this::homeAuto);
    }
//
//    public AutoRoutine leftShoot() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Left Auto");
//        final AutoTrajectory driveRightPath = ChoreoTraj.LeftBasicShootAuto.asAutoTraj(routine);
//
//        routine.active().onTrue(Commands.parallel(
//                driveRightPath.cmd(),
//                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//
//        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(10.0));
//
//        return routine;
//    }
//
//    public AutoRoutine rightShoot() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Right Auto");
//        final AutoTrajectory driveRightPath = ChoreoTraj.RightBasicShootAuto.asAutoTraj(routine);
//
//        routine.active().onTrue(Commands.parallel(
//                driveRightPath.cmd(),
//                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//
//        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(10.0));
//
//        return routine;
//    }
//
//    public AutoRoutine leftSkipBump() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Left Skip Bump Auto");
//        final AutoTrajectory drivePath = ChoreoTraj.LeftSkipBump.asAutoTraj(routine);
//        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
//                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.75, 5.5)));
//
//        final AutoTrajectory drivePath2 = ChoreoTraj.LeftSkipBump2Loop.asAutoTraj(routine);
//
//        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
//                .andThen(() -> m_swerve.resetTranslation(startingTranslation))
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath.cmd())
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(4.75)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath2.cmd()));
//        drivePath2.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(5.0)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));
//
//        return routine;
//    }
//
//    public AutoRoutine rightSkipBump() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Right Skip Bump Auto");
//        final AutoTrajectory drivePath = ChoreoTraj.RightSkipBump.asAutoTraj(routine);
//        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
//                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.75, 2.42)));
//
//        final AutoTrajectory drivePath2 = ChoreoTraj.RightSkipBump2.asAutoTraj(routine);
//
//        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
//                .andThen(() -> m_swerve.resetTranslation(startingTranslation))
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath.cmd())
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(4.75)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath2.cmd()));
//        drivePath2.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(5.0)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));
//
//        return routine;
//    }
//
//    public AutoRoutine leftHotAuto() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Left Skip Bump Hot Auto");
//        final AutoTrajectory drivePath = ChoreoTraj.LeftSkipBump.asAutoTraj(routine);
//        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
//                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.75, 5.5)));
//
//        final AutoTrajectory drivePath2 = ChoreoTraj.LeftSkipBump2Hot.asAutoTraj(routine);
//
//        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
//                .andThen(() -> m_swerve.resetTranslation(startingTranslation))
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath.cmd())
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(5.5)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath2.cmd()));
//
//        return routine;
//    }
//
//    public AutoRoutine rightHotAuto() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Right Skip Bump Hot Auto");
//        final AutoTrajectory drivePath = ChoreoTraj.RightSkipBump.asAutoTraj(routine);
//        final Translation2d startingTranslation = drivePath.getInitialPose().map(Pose2d::getTranslation)
//                .orElseGet(() -> AllianceFlipUtil.apply(new Translation2d(5.7, 2.42)));
//
//        final AutoTrajectory drivePath2 = ChoreoTraj.RightSkipBump2Hot.asAutoTraj(routine);
//
//        routine.active().onTrue(SuperstructureCommands.driveOverBump(m_swerve)
//                .andThen(() -> m_swerve.resetTranslation(startingTranslation))
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath.cmd())
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//        drivePath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator())
//                .withTimeout(5.5)
//                .andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))
//                .andThen(drivePath2.cmd()));
//
//        return routine;
//    }
//
//    public AutoRoutine homeAuto() {
//        final AutoRoutine routine = m_autoFactory.newRoutine("Home Auto");
//        final AutoTrajectory driveRightPath = ChoreoTraj.HomeAuto.asAutoTraj(routine);
//
//        routine.active().onTrue(Commands.parallel(
//                driveRightPath.cmd(),
//                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));
//
//        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()));
//        return routine;
//    }

    private static final Pose2d LEFT_STARTING_POSE = new Pose2d(3.5344300270080566, 5.5548200607299805, Rotation2d.kZero);
    public AutoRoutine blineAuto_Left() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Bline Special Bump Auto Left");

        final Path skipBumpPath = new Path("left_bump_path");
        final Path skipBumpPath2 = new Path("right_bump_p2");
        skipBumpPath.mirror();
        skipBumpPath2.mirror();
        final Command followCommand = m_swerve.getAutoBlineBuilder().build(skipBumpPath);
        final Command followCommand2 = m_swerve.getAutoBlineBuilder().build(skipBumpPath2);

        routine.active().onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    final Pose2d flippedPose = AllianceFlipUtil.apply(LEFT_STARTING_POSE);
                    m_swerve.resetPose(flippedPose);
                }),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 3.0, 0.05),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.5).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.1),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(5.5),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 3.0, 0.05),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.5).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand2),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.1),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(5.5),
                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoRoutine blineAuto_RightSkipBump(final boolean isLeft) {
        final AutoRoutine routine = m_autoFactory.newRoutine("Bline Bump Auto " + (isLeft ? "Left" : "Right"));

        final Path skipBumpPath = new Path("right_skip_bump");
        final Path skipBumpPath2 = new Path("right_bump_p2");
        if (isLeft) {
            skipBumpPath.mirror();
            skipBumpPath2.mirror();
        }

        final Command followCommand = m_swerve.getAutoBlineBuilder().build(skipBumpPath);
        final Command followCommand2 = m_swerve.getAutoBlineBuilder().build(skipBumpPath2);

        routine.active().onTrue(Commands.sequence(
                // use the old cals, no balls in the robot
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 3.0, 0.05)
                        .alongWith(new WaitCommand(0.5).andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))),
//                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.0).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand),
                // drive back with balls
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.06),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(8.0),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                // use the old cals, no balls in the robot
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.05),
                new ParallelDeadlineGroup(
                        new WaitCommand(2.0).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand2),
                // drive back with balls again
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.05),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(5.5),
                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoChooser getAutoChooser() {
        return m_autoChooser;
    }
}
