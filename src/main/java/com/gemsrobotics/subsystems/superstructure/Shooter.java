package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Shooter {
    private static final Distance WHEEL_CIRCUMFERENCE = Inches.of(2).times(2 * Math.PI);
    private static final double SCRUB_FACTOR = 0.7;
    private static final double GEARING = 1.2;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    private final TalonFX m_motorLeader, m_motorFollower;
    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final CoastOut m_coastRequest;
    private final Follower m_followerRequest;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal, m_followerVelocitySignal;
    private final StatusSignal<Voltage> m_leaderVoltsAppliedSignal, m_followerVoltsAppliedSignal;

    private final TalonFXSimState m_leaderSimState, m_followerSimState;
    private final FlywheelSim m_flywheelSim;
    private final Notifier m_simNotifier;

    private boolean m_on;

    public Shooter(final StatusSignalManager signalManager, final TalonFX motorLeader, final TalonFX motorFollower) {
        //region motor config
        m_motorLeader = motorLeader;
        m_motorFollower = motorFollower;

        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 100.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 1.0;
        cfg.MotionMagic.MotionMagicAcceleration = 1000.0;
        m_motorLeader.getConfigurator().apply(cfg);
        m_motorFollower.getConfigurator().apply(cfg);

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0.0);
        m_request.Slot = 0;
        m_followerRequest = new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Opposed);
        m_coastRequest = new CoastOut();
        //endregion

        //region sim code
        m_leaderSimState = m_motorLeader.getSimState();
        m_followerSimState = m_motorFollower.getSimState();

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

        //region logging code
        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVoltsAppliedSignal = m_motorLeader.getMotorVoltage(false);
        m_followerVelocitySignal = m_motorFollower.getVelocity(false);
        m_followerVoltsAppliedSignal = m_motorFollower.getMotorVoltage(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("shooter");
        signalManager.registerPublished(m_leaderVelocitySignal, nt, "leader_velocity_rps");
        signalManager.registerPublished(m_leaderVoltsAppliedSignal, nt, "leader_volts");
        signalManager.registerPublished(m_followerVelocitySignal, nt, "follower_velocity_rps");
        signalManager.registerPublished(m_followerVoltsAppliedSignal, nt, "follower_volts");
        //endregion

        m_on = false;
    }

    public void periodic() {
        m_motorLeader.setControl(m_on ? m_request : m_coastRequest);
        m_motorFollower.setControl(m_followerRequest);
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leaderSimState.getMotorVoltage();
        m_flywheelSim.setInputVoltage(voltage);
        m_flywheelSim.update(SIM_UPDATE_SECONDS);

        m_leaderSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
        m_followerSimState.setRotorVelocity(m_flywheelSim.getAngularVelocity().times(GEARING));
    }

    public void setVelocity(final DoubleSupplier velocitySupplier) {
        m_on = true;
        m_request.Velocity = velocitySupplier.getAsDouble();
    }

    public void setVelocity(final double velocity) {
        setVelocity(() -> velocity);
    }

    public void setOff() {
        m_on = false;
    }

    public double getVelocity() {
        return m_leaderVelocitySignal.getValueAsDouble();
    }

    public LinearVelocity getLaunchVelocity() {
        return MetersPerSecond.of(m_leaderVelocitySignal.getValueAsDouble() * WHEEL_CIRCUMFERENCE.in(Meters) * SCRUB_FACTOR);
    }
}