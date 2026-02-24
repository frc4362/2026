package com.gemsrobotics.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;

import static edu.wpi.first.units.Units.Meters;

public class Flywheel {
    protected final double m_wheelRadiusMeters;
    protected final TalonFX m_motorLeader;
    //private final TalonFX[] m_motorFollowers;

    protected final VelocityTorqueCurrentFOC m_request;
    protected final CoastOut m_coastRequest;

    protected final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
    protected final StatusSignal<Double> m_leaderVelocityReferenceSignal;
    protected final StatusSignal<Current> m_leaderSupplyCurrentSignal, m_leaderStatorCurrentSignal;
    protected final StatusSignal<Temperature> m_leaderTemperatureSignal;

    public Flywheel(final NetworkTable nt,
                    final String ntName,
                    final StatusSignalManager signalManager,
                    final Distance wheelRadius,
                    final TalonFX motorLeader,
                    final TalonFX... motorFollowers) {
        m_wheelRadiusMeters = wheelRadius.in(Meters);
        m_motorLeader = motorLeader;
        //m_motorFollowers = motorFollowers;

        m_request = new VelocityTorqueCurrentFOC(0);

        m_coastRequest = new CoastOut();

        m_leaderVelocitySignal = m_motorLeader.getVelocity(false);
        m_leaderVelocityReferenceSignal = m_motorLeader.getClosedLoopReference(false);
        m_leaderSupplyCurrentSignal = m_motorLeader.getSupplyCurrent(false);
        m_leaderStatorCurrentSignal = m_motorLeader.getStatorCurrent(false);
        m_leaderTemperatureSignal = m_motorLeader.getDeviceTemp(false);

        final NetworkTable myTable = nt.getSubTable(ntName);

        signalManager.registerPublished(m_leaderVelocitySignal, myTable, "velocity_rps");
        signalManager.registerPublished(m_leaderVelocityReferenceSignal, myTable, "velocity_reference_rps");
        signalManager.registerPublished(m_leaderSupplyCurrentSignal, myTable, "supply_current_amps");
        signalManager.registerPublished(m_leaderStatorCurrentSignal, myTable, "stator_current_amps");
        signalManager.registerPublished(m_leaderTemperatureSignal, myTable, "temp_c");
    }

    public Flywheel(final String ntName,
                    final StatusSignalManager signalManager,
                    final Distance wheelRadius,
                    final TalonFX motorLeader,
                    final TalonFX... motorFollowers) {
        this(
                NetworkTableInstance.getDefault().getTable(ntName),
                ntName,
                signalManager,
                wheelRadius,
                motorLeader,
                motorFollowers);
    }

    public void setAngularVelocity(double rps) {
        m_motorLeader.setControl(m_request.withVelocity(rps));
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

    public boolean isAtReference() {
        // angular difference <= 5
        return m_leaderVelocityReferenceSignal.isNear(m_leaderVelocitySignal.getValueAsDouble(), 5.0);
    }
}
