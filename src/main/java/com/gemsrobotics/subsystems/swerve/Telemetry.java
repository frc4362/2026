package com.gemsrobotics.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public final class Telemetry {
    /* Robot swerve drive state */
    private final StructPublisher<Pose2d> m_posePublisher;
    private final StructPublisher<ChassisSpeeds> m_speedsPublisher;
    private final StructArrayPublisher<SwerveModuleState> m_moduleStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> m_moduleTargetsPublisher;
    private final StructArrayPublisher<SwerveModulePosition> m_modulePositionsPublisher;
    private final DoublePublisher m_timestampPublisher;
    private final DoublePublisher m_odometryFrequencyPublisher;

    /**
     * Construct a telemetry object
     */
    public Telemetry(final NetworkTable driveStateTable) {
        SignalLogger.start();

        m_timestampPublisher = driveStateTable.getDoubleTopic("timestamp").publish();
        m_odometryFrequencyPublisher = driveStateTable.getDoubleTopic("odometry_frequency").publish();
        m_posePublisher = driveStateTable.getStructTopic("pose", Pose2d.struct).publish();
        m_speedsPublisher = driveStateTable.getStructTopic("speeds", ChassisSpeeds.struct).publish();
        m_moduleStatesPublisher = driveStateTable.getStructArrayTopic("module_states", SwerveModuleState.struct).publish();
        m_moduleTargetsPublisher = driveStateTable.getStructArrayTopic("module_states_reference", SwerveModuleState.struct).publish();
        m_modulePositionsPublisher = driveStateTable.getStructArrayTopic("module_positions", SwerveModulePosition.struct).publish();
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(final SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        m_posePublisher.set(state.Pose);
        m_speedsPublisher.set(state.Speeds);
        m_timestampPublisher.set(state.Timestamp);
        m_odometryFrequencyPublisher.set(1.0 / state.OdometryPeriod);
        m_moduleStatesPublisher.set(state.ModuleStates);
        m_moduleTargetsPublisher.set(state.ModuleTargets);
        m_modulePositionsPublisher.set(state.ModulePositions);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
    }
}
