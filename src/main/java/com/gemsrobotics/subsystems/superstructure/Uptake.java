package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Constants;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import static java.lang.Math.abs;

public class Uptake {
    private static final double GEARING = 1.0;
    private static final double SIM_UPDATE_SECONDS = 0.001;
    public static final double LAUNCH_COOLDOWN_TIME_SECONDS = 0.5;

    private final TalonFX m_motorLeader, m_motorFollower;
    private final VoltageOut m_volts;
    private final CoastOut m_coastRequest;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal, m_followerVelocitySignal;
    private final StatusSignal<Voltage> m_leaderVoltsAppliedSignal;
    private final StatusSignal<Current> m_leaderStatorCurrentSignal, m_followerStatorCurrentSignal;

    private final TalonFXSimState m_leaderSimState;
    private final FlywheelSim m_rollerSim;
    private final Notifier m_simNotifier;

    private final Trigger m_isLeaderLaunchingTrigger, m_isFollowerLaunchingTrigger;
    public final Trigger isLaunching;

    public Uptake(final StatusSignalManager signalManager, final String ntName, final TalonFX motorLeader, final TalonFX motorFollower) {
        //region motor config
        m_motorLeader = motorLeader;
        m_motorFollower = motorFollower;

        final var cfg = new TalonFXConfiguration();
        cfg.Audio.AllowMusicDurDisable = true;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 90.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 50.0;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
        m_motorLeader.getConfigurator().apply(cfg);
        m_motorFollower.getConfigurator().apply(cfg);

        m_volts = new VoltageOut(0.0);
        m_volts.EnableFOC = true;

        m_coastRequest = new CoastOut();
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
        m_followerVelocitySignal = m_motorFollower.getVelocity(false);
        m_followerStatorCurrentSignal = m_motorFollower.getStatorCurrent(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("uptake").getSubTable(ntName);
        final var powerSignals = signalManager.registerPowerTracking(
                Constants.CAN.kAUX_BUS,
                nt,
                List.of("leader", "follower"),
                List.of(m_motorLeader, m_motorFollower));

        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderVelocitySignal, nt, "velocity_rps");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderVoltsAppliedSignal, nt, "volts");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderStatorCurrentSignal, nt, "stator_amps");
        signalManager.register(Constants.CAN.kAUX_BUS, m_followerVelocitySignal, m_followerStatorCurrentSignal);
        //endregion

        m_isLeaderLaunchingTrigger = new Trigger(
                () -> isUptakeFeedingBalls(m_leaderVelocitySignal.getValueAsDouble(), m_leaderStatorCurrentSignal.getValueAsDouble()))
                    .debounce(LAUNCH_COOLDOWN_TIME_SECONDS, Debouncer.DebounceType.kFalling);
        m_isFollowerLaunchingTrigger = new Trigger(
                () -> isUptakeFeedingBalls(m_followerVelocitySignal.getValueAsDouble(), m_followerStatorCurrentSignal.getValueAsDouble()))
                    .debounce(LAUNCH_COOLDOWN_TIME_SECONDS, Debouncer.DebounceType.kFalling);
        isLaunching = m_isFollowerLaunchingTrigger.or(m_isLeaderLaunchingTrigger);

        m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private static boolean isUptakeFeedingBalls(double velocity, double current) {
        velocity = abs(velocity);
        current = abs(current);

        return velocity < 75.0 && current > 15.0;
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leaderSimState.getMotorVoltage();
        m_rollerSim.setInputVoltage(voltage);
        m_rollerSim.update(SIM_UPDATE_SECONDS);

        m_leaderSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
    }

    public void setFeeding() {
        setVoltage(11.0);
    }

    private void setVoltage(final double volts) {
        m_motorLeader.setControl(m_volts.withOutput(volts));
    }

    public void setIntaking() {
        m_motorLeader.setControl(m_volts.withOutput(-1.0));
    }

    public void setIdle() {
        m_motorLeader.setControl(m_coastRequest);
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
