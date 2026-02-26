package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.Flywheel;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import sun.misc.Signal;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Launcher {
    private static final Distance WHEEL_CIRCUMFERENCE = Inches.of(2).times(2 * Math.PI);
    private static final double SCRUB_FACTOR = 0.7;
    private static final double GEARING = 1;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    private final Flywheel m_wheelLower, m_wheelUpper;

    private final FlywheelSim m_flywheelSim;
    private final Notifier m_simNotifier;

    public Launcher(final Flywheel wheelLower, final Flywheel wheelUpper) {
        m_wheelLower = wheelLower;
        m_wheelUpper = wheelUpper;

        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(2);
        m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, .001, GEARING),
                m_motorModel,
                SIM_UPDATE_SECONDS);

        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(0.001);
        }
        //endregion
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
//        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
//        m_followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
//
//        var voltage = m_leaderSimState.getMotorVoltage();
//        m_flywheelSim.setInputVoltage(voltage);
//        m_flywheelSim.update(SIM_UPDATE_SECONDS);
//
//        m_leaderSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
//        m_followerSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
    }

    public void setLinearVelocity(final double velocity) {
        m_wheelLower.setLinearVelocity(velocity);
        m_wheelUpper.setLinearVelocity(velocity);
    }

    public void setAngularVelocity(final double angularVelocity) {
        m_wheelLower.setAngularVelocity(angularVelocity);
        m_wheelUpper.setAngularVelocity(angularVelocity * 2.2);
    }

    public void setOff() {
        m_wheelLower.setOff();
        m_wheelUpper.setOff();
    }

    public double getAngularVelocity() {
        return m_wheelLower.getAngularVelocity();
    }

    public LinearVelocity getLaunchVelocity() {
        return MetersPerSecond.of(getAngularVelocity() * WHEEL_CIRCUMFERENCE.in(Meters) * SCRUB_FACTOR);
    }

    public boolean isAtReference() {
        return m_wheelLower.isAtReference() && m_wheelUpper.isAtReference();
    }
}