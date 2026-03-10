package com.gemsrobotics.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.subsystems.superstructure.Superstructure;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public final class Autos {

    private final RobotContainer m_robot;
    private final AutoFactory m_autoFactory;
    private final AutoChooser m_autoChooser;

    public Autos(RobotContainer robot) {
        m_robot = robot;
        m_autoFactory = robot.getSwerve().createAutoFactory();
        m_autoFactory
                .bind("Intake", m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.INTAKING))
                .bind("StopIntake", m_robot.getSuperstructure().applyWantedState(Superstructure.SystemState.IDLE));

        m_autoChooser = new AutoChooser();
        m_autoChooser.addRoutine("LeftBasicShootAuto", this::leftShoot);
        m_autoChooser.addRoutine("RightBasicShootAuto", this::rightShoot);
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
        AutoRoutine routine = m_autoFactory.newRoutine("RightBasicShootAuto");

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

    public AutoChooser getAutoChooser() {
        return m_autoChooser;
    }
}
