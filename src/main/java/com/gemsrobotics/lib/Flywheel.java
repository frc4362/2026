package com.gemsrobotics.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.gemsrobotics.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.Map;

import static edu.wpi.first.units.Units.Meters;

public class Flywheel {
    private static final double SIM_UPDATE_SECONDS = 0.001;

    protected final double m_wheelRadiusMeters;
    protected final TalonFX m_motorLeader;
    //private final TalonFX[] m_motorFollowers;

    protected final CANBus m_canBus;
    protected final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest;
    protected final CoastOut m_coastRequest;

    protected final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
    protected final StatusSignal<Double> m_leaderVelocityReferenceSignal;
    protected final StatusSignal<Current> m_leaderSupplyCurrentSignal, m_leaderStatorCurrentSignal;
    protected final StatusSignal<Temperature> m_leaderTemperatureSignal;

    private final TalonFXSimState m_simState;
    private final FlywheelSim m_flywheelSim;
    private final Notifier m_simNotifier;

    public Flywheel(
            final CANBus bus,
            final NetworkTable nt,
            final StatusSignalManager signalManager,
            final Distance wheelRadius,
            final TalonFX motorLeader,
            final TalonFX... motorFollowers
    ) {
        m_canBus = bus;
        m_wheelRadiusMeters = wheelRadius.in(Meters);
        m_motorLeader = motorLeader;
        //m_motorFollowers = motorFollowers;

        m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
        m_coastRequest = new CoastOut();

        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVelocityReferenceSignal = m_motorLeader.getClosedLoopReference(false);
        m_leaderStatorCurrentSignal = m_motorLeader.getStatorCurrent(false);
        m_leaderTemperatureSignal = m_motorLeader.getDeviceTemp(false);

        m_leaderSupplyCurrentSignal = m_motorLeader.getSupplyCurrent(false);

        final NetworkTable myTable = nt;
        signalManager.registerPublished(bus, m_leaderVelocitySignal, myTable, "velocity_rps");
        signalManager.registerPublished(bus, m_leaderVelocityReferenceSignal, myTable, "velocity_reference_rps");
        signalManager.registerPublished(bus, m_leaderStatorCurrentSignal, myTable, "stator_current_amps");
        signalManager.registerPublished(bus, m_leaderSupplyCurrentSignal, myTable, "supply_current_amps");
        signalManager.registerPublished(bus, m_motorLeader.getSupplyVoltage(false), myTable, "supply_voltage");
        signalManager.registerPublished(bus, m_leaderTemperatureSignal, myTable, "temp_c");

        // sim code
        m_simState = m_motorLeader.getSimState();
        final DCMotor m_motorModel = DCMotor.getKrakenX60Foc(1 + motorFollowers.length);
        m_flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(m_motorModel, .007, 1),
                m_motorModel,
                SIM_UPDATE_SECONDS);
        m_simNotifier = new Notifier(this::simulationPeriodic);
        if (Robot.isSimulation()) {
            m_simNotifier.startPeriodic(SIM_UPDATE_SECONDS);
        }
    }

    public Map<Integer, StatusSignalManager.PowerTrackingStatusSignals> attachPowerManagement(final StatusSignalManager signalManager, final NetworkTable table) {
        return signalManager.registerPowerTracking(m_canBus, table, m_motorLeader);
    }

    public void setAngularVelocity(double rps) {
        m_motorLeader.setControl(m_velocityRequest.withVelocity(rps));
//        m_motorLeader.setControl(new VoltageOut(8.0).withEnableFOC(true));
    }

    public void setLinearVelocity(double mps) {
        final double circumference = 2.0 * Math.PI * m_wheelRadiusMeters;
        setAngularVelocity(mps / circumference);
    }

    public void setOff() {
        m_motorLeader.setControl(m_coastRequest);
    }

    public double getAngularVelocity() {
        return m_leaderVelocitySignal.getValueAsDouble();
    }

    public boolean isAtReference(final double tolerance) {
        // angular difference <= 5
        return m_leaderVelocityReferenceSignal.isNear(m_leaderVelocitySignal.getValueAsDouble(), tolerance);
    }

    private void simulationPeriodic() { // Called by the Notifier earlier in this class
        m_simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var voltage = m_simState.getMotorVoltage();
        m_flywheelSim.setInputVoltage(voltage);
        m_flywheelSim.update(SIM_UPDATE_SECONDS);

        m_simState.setRotorVelocity(m_flywheelSim.getAngularVelocity());
        m_simState.setRotorVelocity(m_flywheelSim.getAngularVelocity());
    }

    public TalonFX getMotorLeader() {
        return m_motorLeader;
    }
}
