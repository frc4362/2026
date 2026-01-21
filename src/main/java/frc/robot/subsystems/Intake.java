// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Intake extends SubsystemBase {
//     //TODO: fix values (because these are just copied from last year)
//     private static final double INTAKE_STARTING_ROTATIONS = 0.000;
//     // Assumes the intake is retracted at 0 rotations and deploys in the positive direction
//     private static final double INTAKE_STOWED_ROTATIONS = 3.0;
//     private static final double INTAKE_DEPLOYED_ROTATIONS = 9.000;   // (135deg/360deg)
//     private static final double INTAKE_VOLTAGE = 12;
//     private static final double IDLE_VOLTAGE = 0;

//     private static final double INTAKE_GEARING = 1.0;

//     private final TalonFX m_intake, m_deployer;
//     private final MotionMagicVelocityVoltage m_request;

//     private final StatusSignal<AngularVelocity> m_intakeVelocitySignal;
// 	private final StatusSignal<Voltage> m_intakeVoltsAppliedSignal;
// 	private final DoublePublisher m_intakeVelocityPublisher;
// 	private final DoublePublisher m_intakeVoltsAppliedPublisher;

// 	private final TalonFXSimState m_intakeSimState;
// 	private final DCMotor m_intakeModel;
// 	private final DCMotorSim m_intakeSim;


//     private PositionTorqueCurrentFOC m_positionRequest;
//     private VoltageOut m_voltageRequest;

//     public Intake() {
//         m_intakeModel = DCMotor.getKrakenX60Foc(1);

//         m_intake = new TalonFX(Constants.CAN.INTAKE_TRANSLATION, Constants.CAN.kAUX_BUS);
//         m_deployer = new TalonFX(Constants.CAN.INTAKE_DEPLOYER, Constants.CAN.kAUX_BUS);

//         final var cfg = new TalonFXConfiguration();
//         //TODO: replace values (since these are just copied from 2025OFF)
//         cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
// 		cfg.Feedback.SensorToMechanismRatio = INTAKE_GEARING;
// 		cfg.Slot0.kP = 0.0;
// 		cfg.Slot0.kV = 1.0 / (m_intakeModel.KvRadPerSecPerVolt / (2 * Math.PI));
// 		cfg.Slot0.kA = 0.01;
// 		cfg.MotionMagic.MotionMagicAcceleration = 180.0;
// 		m_intake.getConfigurator().apply(cfg);

//         m_intakeSimState = m_intake.getSimState();
//         m_intakeSim = new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(m_intakeModel, 0.001, INTAKE_GEARING),
//             m_intakeModel
//         );
        
//         m_request = new MotionMagicVelocityVoltage(0.0);
//         m_request.Slot = 0;
//         m_request.Velocity = 0;


//         m_intakeVelocitySignal = m_intake.getVelocity(false);
//         m_intakeVoltsAppliedSignal = m_intake.getMotorVoltage(false);

//         final NetworkTable nt = NetworkTableInstance.getDefault().getTable("intake");
//         m_intakeVelocityPublisher = nt.getDoubleTopic("intake_velocity_rps").publish();
//         m_intakeVoltsAppliedPublisher = nt.getDoubleTopic("intake_volts").publish();
        
//         m_positionRequest = new PositionTorqueCurrentFOC(INTAKE_STARTING_ROTATIONS);
//         m_positionRequest.Slot = 0;
//         m_positionRequest.UseTimesync = false;
//         m_voltageRequest = new VoltageOut(0);
//         m_voltageRequest.UseTimesync = false;
//     }

//     public Command deploy() {
//         return runOnce(
//                 () -> m_deployer.setControl(m_positionRequest.withPosition(INTAKE_DEPLOYED_ROTATIONS))
//         ).withName("Deploy Fuel");
//     }

//     public Command retract() {
//         return runOnce(
//                 () -> m_deployer.setControl(m_positionRequest.withPosition(INTAKE_STOWED_ROTATIONS))
//         ).withName("Retract Fuel");
//     }

//     public Command intake() {
//         return runOnce(
//                 () -> m_intake.setControl(m_voltageRequest.withOutput(INTAKE_VOLTAGE))
//         ).withName("Intake Fuel");
//     }

//     public Command stop() {
//         return runOnce(
//                 () -> m_intake.setControl(m_voltageRequest.withOutput(IDLE_VOLTAGE))
//         ).withName("Stop Intake Fuel");
//     }

//     public Command setIntakeVoltage(final double volts) {
//         return runOnce(() -> {
//             m_intakeSim.setInputVoltage(volts);
//         });
//     }

//     @Override
//     public void simulationPeriodic() {
//         m_intakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

//         var voltage = m_intakeSimState.getMotorVoltage();
//         m_intakeSim.setInputVoltage(voltage);
//         m_intakeSim.update(0.02);

//         m_intakeSimState.setRotorVelocity(m_intakeSim.getAngularVelocity().times(INTAKE_GEARING));
//     }
// }
