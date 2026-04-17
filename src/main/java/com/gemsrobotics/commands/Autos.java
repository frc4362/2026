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
    private final SendableChooser<Boolean> m_isLeftChooser;

    public interface AutoSegment {
        String getBlinePathName();
    }

    public enum FirstAutoSegment implements AutoSegment{
        OutsideIn("pass1_outside_in"),
        InsideOut("pass1_inside_out"),
        OutsideInFull("pass1_alone");

        private final String m_blinePathName;

        FirstAutoSegment(String name) {
            m_blinePathName = name;
        }

        @Override
        public String getBlinePathName() {
            return m_blinePathName;
        }
    }

    public enum SecondAutoSegment implements AutoSegment{
        OutsideIn("pass2_outside_in"),
        InsideOut("pass2_inside_out"),;

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
        m_isLeftChooser = new SendableChooser<>();
        m_isLeftChooser.setDefaultOption("Right", false);
        m_isLeftChooser.addOption("Left", true);
        SmartDashboard.putData(m_isLeftChooser);

        m_autoChooser = new AutoChooser();
        for (final FirstAutoSegment firstPass : FirstAutoSegment.values()) {
            for (final SecondAutoSegment secondPass : SecondAutoSegment.values()) {
                for (final boolean left : new boolean[]{true, false}) {
                    final String side = left ? "Left" : "Right";
                    m_autoChooser.addRoutine(side + ": " + firstPass.name() + "," + secondPass.name(),
                            () -> makeAuto(left, firstPass, secondPass));
                }
            }
        }
    }

    private FollowPath makeFollowPathCommand(final boolean isLeft, final AutoSegment autoSegment) {
        final Path path = new Path(autoSegment.getBlinePathName());
        if (isLeft) {
            path.mirror();
        }

        return m_swerve.getAutoBlineBuilder().build(path);
    }

    private Command followPathUntilBump(final FollowPath pathCommand) {
        return new ParallelDeadlineGroup(
                new WaitCommand(3.0).andThen(new WaitUntilCommand(() -> FieldConstants.isReadyToCrossBump(m_robotState.getLatestFieldToVehicle().getValue()))),
                pathCommand).withTimeout(8.0);
    }

    private static final Pose2d LEFT_STARTING_POSE = new Pose2d(3.5344300270080566, 5.5548200607299805, Rotation2d.kZero);
    private static final double DRIVE_OVER_BUMP_VELOCITY = 4.0;
    private static final double DRIVE_OVER_BUMP_NO_BALLS_DURATION = 0.07;
    private static final double DRIVE_OVER_BUMP_DURATION = 0.19;
    public AutoRoutine makeAuto(final boolean isLeft, final FirstAutoSegment firstAutoSegment, final SecondAutoSegment secondAutoSegment) {
        final AutoRoutine routine = m_autoFactory.newRoutine("configured_auto_" + (isLeft ? "Left" : "Right") + "_" + firstAutoSegment.name() + "_" + secondAutoSegment.name());

        final FollowPath firstPassCommand = makeFollowPathCommand(isLeft, firstAutoSegment);
        final FollowPath secondPassCommand = makeFollowPathCommand(isLeft, secondAutoSegment);

        routine.active().onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    final Pose2d flippedPose = AllianceFlipUtil.apply(LEFT_STARTING_POSE);
                    m_swerve.resetPose(flippedPose);
                }).onlyIf(() -> isLeft),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_NO_BALLS_DURATION),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.SwerveDriveBrake())),
                followPathUntilBump(firstPassCommand),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.SwerveDriveBrake())),
                SuperstructureCommands.launchUntilEmpty(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(7.0),
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY + 1.0, DRIVE_OVER_BUMP_NO_BALLS_DURATION)
                        .beforeStarting(m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)),
                followPathUntilBump(secondPassCommand),
//                // drive back with balls again
                SuperstructureCommands.findAndDriveOverBump(m_robotState, m_swerve, DRIVE_OVER_BUMP_VELOCITY, DRIVE_OVER_BUMP_DURATION),
                m_swerve.runOnce(() -> m_swerve.setControl(new SwerveRequest.SwerveDriveBrake())),
                SuperstructureCommands.launchUntilEmpty(m_swerve, m_superstructure, m_robot.getLaunchCalculator()).withTimeout(6.0),
                m_superstructure.setWantedState(Superstructure.SystemState.IDLE)));

        return routine;
    }

    public AutoChooser getAutoChooser() {
        return m_autoChooser;
    }
}
