package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static java.lang.Math.abs;

public class Hopper {
    private static final double GEARING = 1.0;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    private final TalonFX m_motorLeader, m_motorFollower;
    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final DutyCycleOut m_feedingRequest;
    private final CoastOut m_coastRequest;
    private final Follower m_followerRequest;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal, m_followerVelocitySignal;
    private final StatusSignal<Voltage> m_leaderVoltsAppliedSignal, m_followerVoltsAppliedSignal, m_leaderSupplyVoltageSignal, m_followerSupplyVoltageSignal;
    private final StatusSignal<Current> m_leaderStatorAmpsSignal, m_leaderSupplyAmpsSignal,
            m_followerStatorAmpsSignal, m_followerSupplyAmpsSignal;

    private final TalonFXSimState m_leaderSimState, m_followerSimState;
    private final FlywheelSim m_rollerSim;
    private final Notifier m_simNotifier;

    public final Trigger isHopping;

    public Hopper(final StatusSignalManager signalManager, final TalonFX motorLeader, final TalonFX motorFollower) {
        //region motor config
        m_motorLeader = motorLeader;
        m_motorFollower = motorFollower;

        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 35.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 150.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -150.0;
        cfg.MotionMagic.MotionMagicAcceleration = 500.0;
        m_motorLeader.getConfigurator().apply(cfg);
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 70.0;
        m_motorFollower.getConfigurator().apply(cfg);

        m_feedingRequest = new DutyCycleOut(1.0);
        m_feedingRequest.EnableFOC = true;

        m_coastRequest = new CoastOut();

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0.0);
        m_request.Slot = 0;
        m_followerRequest = new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Aligned);
        //endregion

        //region sim code
        m_leaderSimState = m_motorLeader.getSimState();
        m_followerSimState = m_motorFollower.getSimState();

        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(2);
        m_rollerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, 0.001, GEARING),
                m_motorModel,
                0.01);

        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(SIM_UPDATE_SECONDS);
        }
        //endregion

        //region logging code
        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("hopper");
        final var powerStatusSignals = signalManager.registerPowerTracking(Constants.CAN.kAUX_BUS, nt, m_motorLeader, m_motorFollower);

        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVoltsAppliedSignal = m_motorLeader.getMotorVoltage(false);
        m_leaderStatorAmpsSignal = m_motorLeader.getStatorCurrent(false);
        m_leaderSupplyAmpsSignal = powerStatusSignals.get(m_motorLeader.getDeviceID()).supplyCurrentSignal();
        m_leaderSupplyVoltageSignal = powerStatusSignals.get(m_motorLeader.getDeviceID()).supplyVoltageSignal();
        m_followerVelocitySignal = m_motorFollower.getVelocity(false);
        m_followerVoltsAppliedSignal = m_motorFollower.getMotorVoltage(false);
        m_followerStatorAmpsSignal = m_motorFollower.getStatorCurrent(false);
        m_followerSupplyAmpsSignal = powerStatusSignals.get(m_motorFollower.getDeviceID()).supplyCurrentSignal();
        m_followerSupplyVoltageSignal = powerStatusSignals.get(m_motorFollower.getDeviceID()).supplyVoltageSignal();

        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderVelocitySignal, nt, "leader_velocity_rps");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderVoltsAppliedSignal, nt, "leader_volts");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_leaderStatorAmpsSignal, nt, "leader_stator_amps");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_followerVelocitySignal, nt, "follower_velocity_rps");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_followerVoltsAppliedSignal, nt, "follower_volts");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_followerStatorAmpsSignal, nt, "follower_stator_amps");
        //endregion

        isHopping = new Trigger(() -> {
            final double currentA = abs(m_leaderStatorAmpsSignal.getValueAsDouble());
            final double currentB = abs(m_followerStatorAmpsSignal.getValueAsDouble());
            return (currentA + currentB) > 35;
        })
                .debounce(0.10, Debouncer.DebounceType.kRising)
                .debounce(0.5, Debouncer.DebounceType.kFalling);
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_leaderSimState.getMotorVoltage();
        m_rollerSim.setInputVoltage(voltage);
        m_rollerSim.update(SIM_UPDATE_SECONDS);

        m_leaderSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
        m_followerSimState.setRotorVelocity(m_rollerSim.getAngularVelocity().times(GEARING));
    }

    private void setVelocity(final double velocity) {
        m_request.Velocity = velocity;
    }

    public void setSpitting() {
        m_motorLeader.setControl(new VoltageOut(-6.0));
        m_motorFollower.setControl(new VoltageOut(-6.0));
    }

    public void setFeeding() {
        m_motorLeader.setControl(m_feedingRequest);
        m_motorFollower.setControl(m_feedingRequest);
    }

    public void setIdle() {
        m_motorLeader.setControl(m_coastRequest);
        m_motorFollower.setControl(m_coastRequest);
    }

    public void setIntaking() {
        m_motorLeader.setControl(new VoltageOut(2.0));
        m_motorFollower.setControl(new VoltageOut(2.0));
    }

    public double getVelocity() {
        return m_leaderVelocitySignal.getValueAsDouble();
    }
}
