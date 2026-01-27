package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.function.DoubleSupplier;

public class Uptake {
    private static final double GEARING = 1.0;

    private final TalonFX m_motorLeader, m_motorFollower;
    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final Follower m_followerRequest;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal, m_followerVelocitySignal;
    private final StatusSignal<Voltage> m_leaderVoltsAppliedSignal, m_followerVoltsAppliedSignal;

    private final TalonFXSimState m_leaderSimState, m_followerSimState;
    private final FlywheelSim m_rollerSim;
    private final Notifier m_simNotifier;

    public Uptake(final StatusSignalManager signalManager, final TalonFX motorLeader, final TalonFX motorFollower) {
        //region motor config
        m_motorLeader = motorLeader;
        m_motorFollower = motorFollower;

        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 2.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.MotionMagic.MotionMagicAcceleration = 500.0;
        m_motorLeader.getConfigurator().apply(cfg);
        m_motorFollower.getConfigurator().apply(cfg);

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0.0);
        m_request.Slot = 0;
        m_followerRequest = new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Opposed);
        //endregion

        //region sim code
        m_leaderSimState = m_motorLeader.getSimState();
        m_followerSimState = m_motorFollower.getSimState();

        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(2);
        m_rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, .001, GEARING),
                m_motorModel,
                0.01);

        m_simNotifier = new Notifier(this::simulationPeriodic);
        m_simNotifier.startPeriodic(0.02);
        //endregion

        //region logging code
        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVoltsAppliedSignal = m_motorLeader.getMotorVoltage(false);
        m_followerVelocitySignal = m_motorFollower.getVelocity(false);
        m_followerVoltsAppliedSignal = m_motorFollower.getMotorVoltage(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("uptake");
        signalManager.registerPublished(m_leaderVelocitySignal, nt, "leader_velocity_rps");
        signalManager.registerPublished(m_leaderVoltsAppliedSignal, nt, "leader_volts");
        signalManager.registerPublished(m_followerVelocitySignal, nt, "follower_velocity_rps");
        signalManager.registerPublished(m_followerVoltsAppliedSignal, nt, "follower_volts");
        //endregion
    }

    public void setVelocity(final DoubleSupplier velocitySupplier) {
        m_request.Velocity = velocitySupplier.getAsDouble();
    }

    public void setVelocity(final double velocity) {
        setVelocity(() -> velocity);
    }

    public void setIdle() {
        setVelocity(0);
    }

    public void periodic() {
        m_motorLeader.setControl(m_request);
        m_motorFollower.setControl(m_followerRequest);
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leaderSimState.getMotorVoltage();
        m_rollerSim.setInputVoltage(voltage);
        m_rollerSim.update(0.02);

        m_leaderSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
        m_followerSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
    }
}
