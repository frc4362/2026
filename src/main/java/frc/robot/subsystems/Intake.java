package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    //TODO: fix values (because these are just copied from last year)
    private static final double INTAKE_STARTING_ROTATIONS = 0.000;
    // Assumes the intake is retracted at 0 rotations and deploys in the positive direction
    private static final double INTAKE_STOWED_ROTATIONS = 3.0;
    private static final double INTAKE_DEPLOYED_ROTATIONS = 9.000;   // (135deg/360deg)
    private static final double INTAKE_VOLTAGE = 12;
    private static final double IDLE_VOLTAGE = 0;


    private final TalonFX m_intake, m_deployer;

    private PositionTorqueCurrentFOC m_positionRequest;
    private VoltageOut m_voltageRequest;

    public Intake() {
        m_intake = new TalonFX(Constants.CAN.INTAKE_TRANSLATION, Constants.CAN.kAUX_BUS);
        m_deployer = new TalonFX(Constants.CAN.INTAKE_DEPLOYER, Constants.CAN.kAUX_BUS);
        
        m_positionRequest = new PositionTorqueCurrentFOC(INTAKE_STARTING_ROTATIONS);
        m_positionRequest.Slot = 0;
        m_positionRequest.UseTimesync = false;
        m_voltageRequest = new VoltageOut(0);
        m_voltageRequest.UseTimesync = false;
    }

    public Command deploy() {
        return runOnce(
                () -> m_deployer.setControl(m_positionRequest.withPosition(INTAKE_DEPLOYED_ROTATIONS))
        ).withName("Deploy Fuel");
    }

    public Command retract() {
        return runOnce(
                () -> m_deployer.setControl(m_positionRequest.withPosition(INTAKE_STOWED_ROTATIONS))
        ).withName("Retract Fuel");
    }

    public Command intake() {
        return runOnce(
                () -> m_intake.setControl(m_voltageRequest.withOutput(INTAKE_VOLTAGE))
        ).withName("Intake Fuel");
    }

    public Command stop() {
        return runOnce(
                () -> m_intake.setControl(m_voltageRequest.withOutput(IDLE_VOLTAGE))
        ).withName("Stop Intake Fuel");
    }
}
