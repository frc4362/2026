package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import com.gemsrobotics.Constants;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import java.util.List;

public final class Intake {
//    private static final double INTAKE_STARTING_ROTATIONS = -0.43;
    private static final double INTAKE_STARTING_ROTATIONS = -0.4395;
    // Assumes the intake is retracted at 0 rotations and deploys in the positive direction
    private static final double INTAKE_STOWED_ROTATIONS = -0.31;
    private static final double INTAKE_FEEDING_ROTATIONS = -0.295;
    private static final double INTAKE_AGITATING_ROTATIONS = -0.09;
    private static final double INTAKE_DEPLOYED_ROTATIONS = 0.0;   // (135deg/360deg)
//    private static final double INTAKE_ASSERT_ROTATIONS = 0.5;
    private static final double INTAKE_VELOCITY = 33.0;
    private static final double IDLE_VElOCITY = 0;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    public static final double TRANSLATION_GEARING = 2.62;
    private static final double DEPLOYER_GEARING = 23.0 * (32.0 / 36.0);
    private static final double DEPLOYER_ARM_LENGTH = 0.37;

    private final StatusSignal<AngularVelocity> m_intakeVelocitySignal;
    private final StatusSignal<Current> m_intakeStatorCurrentSignal, m_deployerStatorCurrentSignal,
            m_intakeSupplyCurrentSignal, m_deployerSupplyCurrentSignal;
    private final StatusSignal<Angle> m_deployerPosition;

    private final TalonFXSimState m_intakeSimState;
    private final TalonFXSimState m_deployerSimState;
    private final DCMotor m_intakeModel;
    private final DCMotor m_deployerModel;
    private final DCMotorSim m_intakeSim;
    private final SingleJointedArmSim m_deployerSim;
    private final Notifier m_simNotifier;

    private final VelocityTorqueCurrentFOC m_translationRequest;
    private final DutyCycleOut m_intakingRequest;
    private final DynamicMotionMagicTorqueCurrentFOC m_deployRequest;
    private final TalonFX m_translationLeader, m_translationFollower;
    private final TalonFX m_deployer;

    public Intake(final StatusSignalManager signalManager, final TalonFX intakeLeader, final TalonFX intakeFollower, final TalonFX deployerMotor) {
        m_translationLeader = intakeLeader;
        m_translationFollower = intakeFollower;
        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = TRANSLATION_GEARING;
        cfg.CurrentLimits.StatorCurrentLimit = 120;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Voltage.PeakForwardVoltage = 12;
        cfg.Voltage.PeakReverseVoltage = -12;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kA = 0.0;
        cfg.CurrentLimits.SupplyCurrentLimit = 50.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_translationLeader.getConfigurator().apply(cfg);
        m_translationFollower.getConfigurator().apply(cfg);

        m_deployer = deployerMotor;
        final var cfgDep = new TalonFXConfiguration();
        cfgDep.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgDep.CurrentLimits.StatorCurrentLimit = 80;
        cfgDep.CurrentLimits.StatorCurrentLimitEnable = true;
        cfgDep.CurrentLimits.SupplyCurrentLimit = 40;
        cfgDep.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfgDep.Voltage.PeakForwardVoltage = 12;
        cfgDep.Voltage.PeakReverseVoltage = -12;
        cfgDep.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfgDep.Slot0.kP = 3000;
        cfgDep.Slot0.kD = 30;
        cfgDep.MotionMagic.MotionMagicAcceleration = 2.0;
        cfgDep.Feedback.SensorToMechanismRatio = DEPLOYER_GEARING;
        m_deployer.getConfigurator().apply(cfgDep);

        m_deployer.setPosition(INTAKE_STARTING_ROTATIONS);

        m_intakingRequest = new DutyCycleOut(1.0);
        m_intakingRequest.EnableFOC = true;

        m_translationRequest = new VelocityTorqueCurrentFOC(0);
        m_deployRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);
        m_deployRequest.Acceleration = 7.0; // lol

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("intake");
        final var powerStatusSignals = signalManager.registerPowerTracking(
                Constants.CAN.kAUX_BUS,
                nt,
                List.of("deployer", "roller_top", "roller_bot"),
                List.of(m_deployer, m_translationLeader, m_translationFollower));

        m_intakeVelocitySignal = m_translationLeader.getVelocity(false);
        m_intakeStatorCurrentSignal = m_translationLeader.getStatorCurrent(false);
        m_deployerStatorCurrentSignal = m_deployer.getStatorCurrent(false);
        m_intakeSupplyCurrentSignal = powerStatusSignals.get(m_translationLeader.getDeviceID()).supplyCurrentSignal();
        m_deployerSupplyCurrentSignal = powerStatusSignals.get(m_deployer.getDeviceID()).supplyCurrentSignal();
        m_deployerPosition = m_deployer.getPosition(false);

        // do this so the follower stays tapped in
        m_translationLeader.getTorqueCurrent(false).setUpdateFrequency(250.0);

        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_intakeVelocitySignal, nt, "intake_velocity_rps");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_intakeStatorCurrentSignal, nt, "intake_stator_current");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_deployerStatorCurrentSignal, nt, "deployer_stator_current");
//        signalManager.registerPublished(m_intakeSupplyCurrentSignal, nt, "intake_supply_current");
//        signalManager.registerPublished(m_deployerSupplyCurrentSignal, nt, "deployer_supply_current");
        signalManager.registerPublished(Constants.CAN.kAUX_BUS, m_deployerPosition, nt, "deployer_position");

        // sim code
        m_intakeModel = DCMotor.getKrakenX60Foc(1);
        m_deployerModel = DCMotor.getKrakenX44Foc(1);

        m_intakeSimState = m_translationLeader.getSimState();
        m_intakeSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(m_intakeModel, 0.001, TRANSLATION_GEARING),
                m_intakeModel
        );

        m_deployerSimState = m_deployer.getSimState();
        m_deployerSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(m_deployerModel, 0.01, DEPLOYER_GEARING),
                m_deployerModel,
                DEPLOYER_GEARING,
                DEPLOYER_ARM_LENGTH,
                0.0,
                Math.toRadians(105),
                true,
                0.0,
                0.0, 0.0
        );

        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(0.001);
        }
    }

    public void setDeploy() {
        m_deployer.setControl(m_deployRequest
                .withPosition(INTAKE_DEPLOYED_ROTATIONS)
                .withVelocity(10));
    }

    public void setRetract() {
        m_deployer.setControl(m_deployRequest
                .withPosition(INTAKE_STOWED_ROTATIONS)
                .withVelocity(10));
    }

    public void setRetractSlowly() {
        m_deployer.setControl(m_deployRequest
                .withPosition(INTAKE_FEEDING_ROTATIONS)
                .withVelocity(1.1 / 6.0));
    }

    public void setAgitating() {
        m_deployer.setControl(m_deployRequest
                .withPosition(INTAKE_AGITATING_ROTATIONS)
                .withVelocity(15));
    }

    public void setPushing() {
        m_deployer.setControl(m_deployRequest.withPosition(-0.17).withVelocity(15));
    }

    public void setFeedingHopper() {
        m_translationLeader.setControl(m_translationRequest.withVelocity(INTAKE_VELOCITY / 3.0));
        m_translationFollower.setControl(m_translationRequest.withVelocity(INTAKE_VELOCITY / 3.0));
    }

    public void setIntaking() {
        m_translationLeader.setControl(m_intakingRequest);
        m_translationFollower.setControl(m_intakingRequest);
    }

    public void setSpitting() {
        m_translationLeader.setControl(m_translationRequest.withVelocity(-INTAKE_VELOCITY));
        m_translationFollower.setControl(m_translationRequest.withVelocity(-INTAKE_VELOCITY));
    }

    public void setStop() {
        m_translationLeader.setControl(new CoastOut());
        m_translationFollower.setControl(new CoastOut());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_deployerPosition.getValueAsDouble());
    }

    public void simulationPeriodic() {
        m_intakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_deployerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_intakeSim.setInputVoltage(m_intakeSimState.getMotorVoltage());
        m_intakeSim.update(SIM_UPDATE_SECONDS);

        m_deployerSim.setInputVoltage(m_deployerSimState.getMotorVoltage());
        m_deployerSim.update(SIM_UPDATE_SECONDS);

        m_intakeSimState.setRotorVelocity(m_intakeSim.getAngularVelocity().times(TRANSLATION_GEARING));
        m_intakeSimState.setRawRotorPosition(m_intakeSim.getAngularPosition().times(TRANSLATION_GEARING));
        m_deployerSimState.setRawRotorPosition(m_deployerSim.getAngleRads() / 2 * Math.PI * DEPLOYER_GEARING);
        m_deployerSimState.setRotorVelocity(m_deployerSim.getAngleRads() / 2 * Math.PI * DEPLOYER_GEARING);
    }
}
