package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class Intake {
    //TODO: fix values (because these are just copied from last year)
    private static final double INTAKE_STARTING_ROTATIONS = -0.43;
    // Assumes the intake is retracted at 0 rotations and deploys in the positive direction
    private static final double INTAKE_STOWED_ROTATIONS = -0.4;
    private static final double INTAKE_FEEDING_ROTATIONS = -0.295;
    private static final double INTAKE_DEPLOYED_ROTATIONS = 0.0;   // (135deg/360deg)
//    private static final double INTAKE_ASSERT_ROTATIONS = 0.5;
    private static final double INTAKE_VELOCITY = 30;
    private static final double IDLE_VElOCITY = 0;
    private static final double SIM_UPDATE_SECONDS = 0.001;

    private static final double INTAKE_GEARING = 3.0 / 1.0;
    private static final double DEPLOYER_GEARING = 23.0 * (32.0 / 36.0);
    private static final double DEPLOYER_ARM_LENGTH = 0.37;

    private final StatusSignal<AngularVelocity> m_intakeVelocitySignal;
    private final StatusSignal<Current> m_intakeStatorCurrentSignal, m_deployerStatorCurrentSignal, m_intakeSupplyCurrentSignal, m_deployerSupplyCurrentSignal;
    private final StatusSignal<Angle> m_deployerPosition;

    private final TalonFXSimState m_intakeSimState;
    private final TalonFXSimState m_deployerSimState;
    private final DCMotor m_intakeModel;
    private final DCMotor m_deployerModel;
    private final DCMotorSim m_intakeSim;
    private final SingleJointedArmSim m_deployerSim;
    private final Notifier m_simNotifier;

    private final VelocityTorqueCurrentFOC m_request;
    private final DynamicMotionMagicTorqueCurrentFOC m_deployRequest;
    private final TalonFX m_intakeLeader, m_intakeFollower;
    private final TalonFX m_intakeDeployer;

    public Intake(final StatusSignalManager signalManager, final TalonFX intakeLeader, final TalonFX intakeFollower, final TalonFX deployerMotor) {
        m_intakeLeader = intakeLeader;
        m_intakeFollower = intakeFollower;
        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = INTAKE_GEARING;
        cfg.CurrentLimits.StatorCurrentLimit = 120;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.Voltage.PeakForwardVoltage = 12;
        cfg.Voltage.PeakReverseVoltage = -12;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kA = 0.0;
        m_intakeLeader.getConfigurator().apply(cfg);
        m_intakeFollower.getConfigurator().apply(cfg);

        m_intakeDeployer = deployerMotor;
        final var cfgDep = new TalonFXConfiguration();
        cfgDep.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgDep.CurrentLimits.StatorCurrentLimit = 60;
        cfgDep.CurrentLimits.StatorCurrentLimitEnable = true;
        cfgDep.Voltage.PeakForwardVoltage = 12;
        cfgDep.Voltage.PeakReverseVoltage = -12;
        cfgDep.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfgDep.Slot0.kP = 3000;
        cfgDep.Slot0.kD = 30;
        cfgDep.MotionMagic.MotionMagicAcceleration = 1.0;
        cfgDep.Feedback.SensorToMechanismRatio = DEPLOYER_GEARING;
        m_intakeDeployer.getConfigurator().apply(cfgDep);

        m_intakeDeployer.setPosition(INTAKE_STARTING_ROTATIONS);

        m_request = new VelocityTorqueCurrentFOC(0);
        m_deployRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);
        m_deployRequest.Acceleration = 7;

        m_intakeVelocitySignal = m_intakeLeader.getVelocity(false);
        m_intakeStatorCurrentSignal = m_intakeLeader.getStatorCurrent(false);
        m_deployerStatorCurrentSignal = m_intakeDeployer.getStatorCurrent(false);
        m_intakeSupplyCurrentSignal = m_intakeLeader.getSupplyCurrent(false);
        m_deployerSupplyCurrentSignal = m_intakeDeployer.getSupplyCurrent(false);
        m_deployerPosition = m_intakeDeployer.getPosition(false);

        // do this so the follower stays tapped in
        m_intakeLeader.getTorqueCurrent(false).setUpdateFrequency(250.0);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("intake");
        signalManager.registerPublished(m_intakeVelocitySignal, nt, "intake_velocity_rps");
        signalManager.registerPublished(m_intakeStatorCurrentSignal, nt, "intake_stator_current");
        signalManager.registerPublished(m_deployerStatorCurrentSignal, nt, "deployer_stator_current");
        signalManager.registerPublished(m_intakeSupplyCurrentSignal, nt, "intake_supply_current");
        signalManager.registerPublished(m_deployerSupplyCurrentSignal, nt, "deployer_supply_current");
        signalManager.registerPublished(m_deployerPosition, nt, "deployer_position");

        // sim code
        m_intakeModel = DCMotor.getKrakenX60Foc(1);
        m_deployerModel = DCMotor.getKrakenX44Foc(1);

        m_intakeSimState = m_intakeLeader.getSimState();
        m_intakeSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(m_intakeModel, 0.001, INTAKE_GEARING),
                m_intakeModel
        );

        m_deployerSimState = m_intakeDeployer.getSimState();
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
        m_intakeDeployer.setControl(m_deployRequest
                .withPosition(INTAKE_DEPLOYED_ROTATIONS)
                .withVelocity(10));
    }

    public void setRetract() {
        m_intakeDeployer.setControl(m_deployRequest
                .withPosition(INTAKE_STOWED_ROTATIONS)
                .withVelocity(10));
    }

    public void setRetractSlowly() {
        m_intakeDeployer.setControl(m_deployRequest
                .withPosition(INTAKE_FEEDING_ROTATIONS)
                .withVelocity(1.1 / 6.0));
    }

    public void setIntaking() {
        m_intakeLeader.setControl(new DutyCycleOut(1.0));
        m_intakeFollower.setControl(new DutyCycleOut(1.0));
//        m_intakeLeader.setControl(m_request.withVelocity(INTAKE_VELOCITY));
//        m_intakeFollower.setControl(m_request.withVelocity(INTAKE_VELOCITY));
    }

    public void setSpitting() {
        m_intakeLeader.setControl(m_request.withVelocity(-INTAKE_VELOCITY));
        m_intakeFollower.setControl(m_request.withVelocity(-INTAKE_VELOCITY));
    }

    public void setStop() {
        m_intakeLeader.setControl(new CoastOut());
        m_intakeFollower.setControl(new CoastOut());
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

        m_intakeSimState.setRotorVelocity(m_intakeSim.getAngularVelocity().times(INTAKE_GEARING));
        m_deployerSimState.setRawRotorPosition(m_deployerSim.getAngleRads() / 2 * Math.PI * DEPLOYER_GEARING);
    }

//    private boolean hasAssertedThisIntake = false;
//    public void assertDeployed(Timer timer) {
//        if (timer.hasElapsed(.5)) {
//            if (!hasAssertedThisIntake) {
//                m_intakeDeployer.setPosition(INTAKE_DEPLOYED_ROTATIONS);
//                hasAssertedThisIntake = true;
//            }
//            setDeploy();
//        } else {
//            m_intakeDeployer.setControl(m_deployRequest
//                    .withPosition(INTAKE_ASSERT_ROTATIONS)
//                    .withVelocity(10));
//            hasAssertedThisIntake = false;
//        }
//    }
}

