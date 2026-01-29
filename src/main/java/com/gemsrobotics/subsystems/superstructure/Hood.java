package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.StatusSignalManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class Hood {
    private static final double GEARING = 300.0;
    // 15 degrees forward from the vertical, or 75 degrees up from the horizon
    public static final Rotation2d MIN_ANGLE  = Rotation2d.fromDegrees(15.0);
    public static final Rotation2d MAX_ANGLE  = Rotation2d.fromDegrees(55.0);
    private static final Rotation2d DEFAULT_TOLERANCE = Rotation2d.fromDegrees(0.5);

    private final TalonFX m_motor;
    private final PositionTorqueCurrentFOC m_request;

    private final StatusSignal<Angle> m_motorRotations;
    private final StatusSignal<Current> m_motorAmps;
    private final StructPublisher<Rotation2d> m_worldAnglePublisher;

    private Rotation2d m_reference;

    // sim
    private final TalonFXSimState m_simState;
    private final Notifier m_simNotifier;
    private final DCMotorSim m_motorModel;
    private static final double SIM_PERIOD = 0.001;

    public Hood(final StatusSignalManager signalManager, final TalonFX motor) {
        m_motor = motor;

        final var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Feedback.SensorToMechanismRatio = GEARING;
        cfg.Slot0.kP = 20.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -60.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // this should be the maximum rotor position later
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(cfg);

        m_request = new PositionTorqueCurrentFOC(0.0);
        m_request.Slot = 0;

        m_motorRotations = m_motor.getRotorPosition(false);
        m_motorAmps = m_motor.getTorqueCurrent(false);

        final NetworkTable nt = NetworkTableInstance.getDefault().getTable("hood");
        m_worldAnglePublisher = nt.getStructTopic("rotations_world", Rotation2d.struct).publish();
        signalManager.registerPublished(m_motorRotations, nt, "rotations_motor");
        signalManager.registerPublished(m_motorAmps, nt, "amps");

        // sim
        m_motorModel = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.001, GEARING),
                DCMotor.getKrakenX44Foc(1));

        m_simState = m_motor.getSimState();
        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(SIM_PERIOD);
        }
    }

    public void periodic() {
        m_worldAnglePublisher.set(rotor2WorldAngle(m_motorRotations.getValueAsDouble()));
        m_motor.setControl(m_request.withPosition(angle2Rotor(m_reference)));
    }

    public void setReference(final Rotation2d worldAngle) {
        m_reference = clampAngle(worldAngle);
    }

    public boolean atReference(final Rotation2d tolerance) {
        return m_motorRotations.isNear(angle2Rotor(m_reference), tolerance.getRotations());
    }

    public boolean atReference() {
        return atReference(DEFAULT_TOLERANCE);
    }

    private Rotation2d rotor2WorldAngle(final double rotorValue) {
        return Rotation2d.fromRotations(rotorValue).plus(MIN_ANGLE);
    }

    private double angle2Rotor(final Rotation2d worldAngle) {
        return worldAngle.minus(MIN_ANGLE).getRotations();
    }

    private Rotation2d clampAngle(final Rotation2d worldAngle) {
        return Rotation2d.fromRotations(MathUtil.clamp(worldAngle.getRotations(), MIN_ANGLE.getRotations(), MAX_ANGLE.getRotations()));
    }

    private void simulationPeriodic() {
        m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_simState.getMotorVoltage();
        m_motorModel.setInputVoltage(voltage);
        m_motorModel.update(SIM_PERIOD);

        m_simState.setRawRotorPosition(m_motorModel.getAngularPosition().times(GEARING));
        m_simState.setRotorVelocity(m_motorModel.getAngularVelocity().times(GEARING));
    }
}
