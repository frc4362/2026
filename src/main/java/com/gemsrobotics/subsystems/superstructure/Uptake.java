package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.List;
import java.util.function.DoubleSupplier;

public class Uptake {
    private static final double GEARING = 1.0;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    private final TalonFX m_motorLeader, m_motorFollower;
    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final VoltageOut m_volts;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
    private final StatusSignal<Voltage> m_leaderVoltsAppliedSignal;
    private final StatusSignal<Current> m_leaderStatorCurrentSignal;

    private final TalonFXSimState m_leaderSimState;
    private final FlywheelSim m_rollerSim;
    private final Notifier m_simNotifier;

    public Uptake(final StatusSignalManager signalManager, final String ntName, final TalonFX motorLeader, final TalonFX motorFollower) {
        //region motor config
        m_motorLeader = motorLeader;
        m_motorFollower = motorFollower;

        final var cfg = new TalonFXConfiguration();
        cfg.Audio.AllowMusicDurDisable = true;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 80.0;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 2.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.MotionMagic.MotionMagicAcceleration = 500.0;
        m_motorLeader.getConfigurator().apply(cfg);
        m_motorFollower.getConfigurator().apply(cfg);

        m_volts = new VoltageOut(0.0);
        m_volts.EnableFOC = true;

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0.0);
        m_request.Slot = 0;
        //endregion

        //region sim code
        m_leaderSimState = m_motorLeader.getSimState();

        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(1);
        m_rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, .001, GEARING),
                m_motorModel,
                0.01);

        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(SIM_UPDATE_SECONDS);
        }
        //endregion

        //region logging code
        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVoltsAppliedSignal = m_motorLeader.getMotorVoltage(false);
        m_leaderVoltsAppliedSignal.setUpdateFrequency(250);
        m_leaderStatorCurrentSignal = m_motorLeader.getStatorCurrent(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("uptake").getSubTable(ntName);
        final var powerSignals = signalManager.registerPowerTracking(
                nt,
                List.of("leader", "follower"),
                List.of(m_motorLeader, m_motorFollower));

        signalManager.registerPublished(m_leaderVelocitySignal, nt, "velocity_rps");
        signalManager.registerPublished(m_leaderVoltsAppliedSignal, nt, "volts");
        signalManager.registerPublished(m_leaderStatorCurrentSignal, nt, "stator_amps");
        //endregion

        m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leaderSimState.getMotorVoltage();
        m_rollerSim.setInputVoltage(voltage);
        m_rollerSim.update(SIM_UPDATE_SECONDS);

        m_leaderSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
    }

    public void setVoltage(final double volts) {
        m_motorLeader.setControl(m_volts.withOutput(volts));
    }

    public void setIdle() {
        m_motorLeader.setControl(new CoastOut());
    }

    public double getVelocity() {
        return m_leaderVelocitySignal.getValueAsDouble();
    }

    public TalonFX getLeaderMotor() {
        return m_motorLeader;
    }

    public TalonFX getFollowerMotor() {
        return m_motorFollower;
    }
}
