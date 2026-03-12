package com.gemsrobotics.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.choreo.ChoreoTraj;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;

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
        m_autoChooser.addRoutine("Right Auto", this::rightShoot);
        m_autoChooser.addRoutine("Home Auto", this::homeAuto);
    }

    public AutoRoutine leftShoot() {
        AutoRoutine routine = m_autoFactory.newRoutine("LeftBasicShootAuto");
        AutoTrajectory leftShoot = routine.trajectory("LeftBasicShootAuto");

        routine.active().onTrue(Commands.parallel(
                run(() -> m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.INTAKING))
                        .andThen(m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE)),
                leftShoot.cmd()
        ));
        leftShoot.done().onTrue(Commands.sequence(
//                new AimAndBrakeCommand(m_robot.getDrivetrain()),
//                m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.LAUNCHING),
//                new WaitCommand(10), // Replace this with WaitUntilCommand([hopper empty])
//                m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE)
        ));

        return routine;
    }

    public AutoRoutine rightShoot() {
        final AutoRoutine routine = m_autoFactory.newRoutine("Right Auto");
        final AutoTrajectory driveRightPath = ChoreoTraj.RightBasicShootAuto.asAutoTraj(routine);

        routine.active().onTrue(Commands.parallel(
                driveRightPath.cmd(),
                m_superstructure.setWantedState(Superstructure.SystemState.INTAKING)));

        driveRightPath.done().onTrue(SuperstructureCommands.makeLaunchCommand(m_swerve, m_superstructure, m_robot.getLaunchCalculator()));

//        AutoTrajectory rightShoot = routine.trajectory("RightBasicShootAuto");
//        routine.active().onTrue(Commands.parallel(
//                        run(() -> m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.INTAKING))
//                        .andThen(m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE)),
//                        rightShoot.cmd()
//        ));
//        rightShoot.done().onTrue(Commands.sequence(
//                new AimAndBrakeCommand(m_robot.getDrivetrain()),
//                m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.LAUNCHING),
//                new WaitCommand(10), // Replace this with WaitUntilCommand([hopper empty])
//                m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE)
//        ));

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
