package com.gemsrobotics.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;

import static edu.wpi.first.units.Units.Meters;

public class Flywheel {
    private final double m_wheelRadiusMeters;
    private final TalonFX m_motorLeader;
    //private final TalonFX[] m_motorFollowers;

    private final MotionMagicVelocityTorqueCurrentFOC m_request;
    private final CoastOut m_coastRequest;

    private final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
    private final StatusSignal<Double> m_leaderVelocityReferenceSignal;
    private final StatusSignal<Current> m_leaderSupplyCurrentSignal, m_leaderStatorCurrentSignal;
    private final StatusSignal<Temperature> m_leaderTemperatureSignal;

    public Flywheel(final NetworkTable nt,
                    final String ntName,
                    final StatusSignalManager signalManager,
                    final Distance wheelRadius,
                    final TalonFX motorLeader,
                    final TalonFX... motorFollowers) {
        m_wheelRadiusMeters = wheelRadius.in(Meters);
        m_motorLeader = motorLeader;
        //m_motorFollowers = motorFollowers;

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

        m_coastRequest = new CoastOut();

        m_leaderVelocitySignal = m_motorLeader.getVelocity();
        m_leaderVelocityReferenceSignal = m_motorLeader.getClosedLoopReference();
        m_leaderSupplyCurrentSignal = m_motorLeader.getSupplyCurrent();
        m_leaderStatorCurrentSignal = m_motorLeader.getStatorCurrent();
        m_leaderTemperatureSignal = m_motorLeader.getDeviceTemp();

        signalManager.registerPublished(m_leaderVelocitySignal, nt, "leader_velocity_rps");
        signalManager.registerPublished(m_leaderVelocityReferenceSignal, nt, "leader_velocity_reference_rps");
        signalManager.registerPublished(m_leaderSupplyCurrentSignal, nt, "leader_supply_current_amps");
        signalManager.registerPublished(m_leaderStatorCurrentSignal, nt, "leader_stator_current_amps");
        signalManager.registerPublished(m_leaderTemperatureSignal, nt, "leader_temp_c");
    }

    public Flywheel(final String ntName,
                    final StatusSignalManager signalManager,
                    final Distance wheelRadius,
                    final TalonFX motorLeader,
                    final TalonFX... motorFollowers) {
        this(
                NetworkTableInstance.getDefault().getTable(ntName + "/" + ntName),
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
}
