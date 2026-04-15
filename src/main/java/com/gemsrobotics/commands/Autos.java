package com.gemsrobotics.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.lib.EnumChooser;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

import static java.lang.Math.abs;

public final class Autos {
    private final RobotContainer m_robot;
    private final RobotState m_robotState;
    private final Superstructure m_superstructure;
    private final CommandSwerveDrivetrain m_swerve;
    private final AutoFactory m_autoFactory;
    private final AutoChooser m_autoChooser;
    private final SendableChooser<FirstAutoSegment> m_autoSegment1Chooser;
    private final SendableChooser<SecondAutoSegment> m_autoSegment2Chooser;

    public interface AutoSegment {
        String getBlinePathName();
    }

    public enum FirstAutoSegment implements AutoSegment{
        OutsideIn("right_skip_bump"),
        InsideOut("right_skip_bump_cw"),
        OutsideInFull("right_skip_bump_alone"),
        DoubleTrench("double_trench_auto");

        private final String name;

        FirstAutoSegment(String name) {
            this.name = name;
        }

        @Override
        public String getBlinePathName() {
            return name;
        }
    }

    public enum SecondAutoSegment implements AutoSegment{
        OutsideIn("pass2_outside_in"),
        InsideOut("right_bump_p2");

        private final String m_blinePathName;

        SecondAutoSegment(final String blinePathName) {
            m_blinePathName = blinePathName;
        }

        @Override
        public String getBlinePathName() {
            return m_blinePathName;
        }
    }

    public Autos(final RobotContainer robot) {
        m_robot = robot;
        m_autoFactory = robot.getSwerve().createAutoFactory();

        m_robotState = m_robot.getRobotState();
        m_superstructure = m_robot.getSuperstructure();
        m_swerve = m_robot.getSwerve();

        m_autoSegment1Chooser = new EnumChooser<>(FirstAutoSegment.class, FirstAutoSegment.OutsideIn);
        SmartDashboard.putData(m_autoSegment1Chooser);
        m_autoSegment2Chooser = new EnumChooser<>(SecondAutoSegment.class, SecondAutoSegment.InsideOut);
        SmartDashboard.putData(m_autoSegment2Chooser);
        final SendableChooser<Boolean> isLeftChooser = new SendableChooser<>();
        isLeftChooser.setDefaultOption("Right", false);
        isLeftChooser.addOption("Left", true);
        SmartDashboard.putData(isLeftChooser);

        m_autoChooser = new AutoChooser();
        m_autoChooser.addRoutine("Bline Right Auto", () -> blineAuto_RightSkipBump(false));
        m_autoChooser.addRoutine("Bline Left Auto", this::blineAuto_Left);
        m_autoChooser.addRoutine("Configurable Auto", () -> makeAuto(
                isLeftChooser.getSelected(),
                m_autoSegment1Chooser.getSelected(),
                m_autoSegment2Chooser.getSelected()));
    }

    private FollowPath makeFollowPathCommand(final boolean isLeft, final AutoSegment autoSegment) {
        return m_swerve.getAutoBlineBuilder().build(new Path((autoSegment.getBlinePathName())));
    }

    private Command followPathUntilBump(final FollowPath pathCommand) {
        return new ParallelDeadlineGroup(
                new WaitCommand(1.0).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                pathCommand);
    }

    private static final Pose2d LEFT_STARTING_POSE = new Pose2d(3.5344300270080566, 5.5548200607299805, Rotation2d.kZero);
    private static final double DRIVE_OVER_BUMP_VELOCITY = 4.0;
    private static final double DRIVE_OVER_BUMP_DURATION = 0.04;
    public AutoRoutine makeAuto(final boolean isLeft, final FirstAutoSegment firstAutoSegment, final SecondAutoSegment secondAutoSegment) {
        final AutoRoutine routine = m_autoFactory.newRoutine("configured_auto");

        final FollowPath firstPassCommand = makeFollowPathCommand(isLeft, firstAutoSegment);
        final FollowPath secondPassCommand = makeFollowPathCommand(isLeft, secondAutoSegment);

        routine.active().onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    final Pose2d flippedPose = AllianceFlipUtil.apply(LEFT_STARTING_POSE);
                    m_swerve.resetPose(flippedPose);
                }).onlyIf(() -> isLeft),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION)
                        .alongWith(new WaitCommand(0.4).andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))),
                followPathUntilBump(firstPassCommand),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION),
                SuperstructureCommands.launchUntilEmpty(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(6.0),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION),
                followPathUntilBump(secondPassCommand),
//                // drive back with balls again
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.launchUntilEmpty(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(6.0),
                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }
    
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
                        new WaitCommand(1.75).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, 0.06),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(4.5),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, 0.05),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.75).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand2),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, 0.05),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(4.5),
                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoRoutine blineAuto_RightSkipBump(final boolean isLeft) {
        final AutoRoutine routine = m_autoFactory.newRoutine("Bline Bump Auto " + (isLeft ? "Left" : "Right"));

        final Path skipBumpPath = new Path("right_skip_bump_cw");
        final Path skipBumpPath2 = new Path("right_bump_p2");
        if (isLeft) {
            skipBumpPath.mirror();
            skipBumpPath2.mirror();
        }

        final Command followCommand = m_swerve.getAutoBlineBuilder().build(skipBumpPath);
        final Command followCommand2 = m_swerve.getAutoBlineBuilder().build(skipBumpPath2);

        routine.active().onTrue(Commands.sequence(
                // use the old cals, no balls in the robot
//                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 3.0, 0.05)
//                        .alongWith(new WaitCommand(0.5).andThen(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING))),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.0).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                        followCommand),
                // drive back with balls
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, 0.05),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
                SuperstructureCommands.launchUntilEmpty(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(8.0),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                // use the old cals, no balls in the robot
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, 0.05),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle()))));
//                new ParallelDeadlineGroup(
//                        new WaitCommand(1.5).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
//                        followCommand2),
//                // drive back with balls again
//                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, 4.0, 0.05),
//                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.Idle())),
//                SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(5.5),
//                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoChooser getAutoChooser() {
        return m_autoChooser;
    }
}
