package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Robot;
import com.gemsrobotics.lib.StatusSignalManager;
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

public class Hood {
    private static final double GEARING = 150.0;
    // 15 degrees forward from the vertical
    public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(15.0);
    private static final double SIM_PERIOD = 0.001;

    private final TalonFX m_motor;
    private final PositionTorqueCurrentFOC m_request;

    // sim
    private final TalonFXSimState m_simState;
    private final Notifier m_simNotifier;
    private final DCMotorSim m_motorModel;

    private final StatusSignal<Angle> m_motorRotations;
    private final StatusSignal<Current> m_motorAmps;
    private final StructPublisher<Rotation2d> m_worldAnglePublisher;

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
        m_worldAnglePublisher.set(getWorldAngle());
    }

    public void setWorldAngle(final Rotation2d goal) {
        m_motor.setControl(m_request.withPosition(angle2Rotor(goal)));
    }

    private double angle2Rotor(final Rotation2d angle) {
        return angle.getRotations();
    }

    private Rotation2d rotor2Angle(final double rotations) {
        return Rotation2d.fromRotations(rotations);
    }

    public Rotation2d getWorldAngle() {
        return rotor2Angle(m_motorRotations.getValueAsDouble()).plus(STARTING_ANGLE);
    }

    private double clampSafeRotor(final double rotorPosition) {
        return 0.0;
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
