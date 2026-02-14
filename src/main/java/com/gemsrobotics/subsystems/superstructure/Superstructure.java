package com.gemsrobotics.subsystems.superstructure;

import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.launching.*;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Superstructure extends SubsystemBase {
    private static final String NT_KEY = "superstructure";

    public enum SystemState {
        IDLE,
        INTAKING,
        LAUNCHING,
        CLIMBING,
        CLIMBED
    }

    private final CommandSwerveDrivetrain m_swerve;
    private final Launcher m_launcher;
    private final Hopper m_hopper;
    private final Uptake m_uptake;
    private final Hood m_hood;
    private final Intake m_intake;

    private final StringPublisher m_systemStatePublisher;
    private final StringPublisher m_wantedStatePublisher;
    private final DoublePublisher m_hubDistancePublisher, m_launchVelocityPublisher;
    private final StructPublisher<Rotation2d> m_launchAnglePublisher;

    private SystemState m_state;
    private SystemState m_stateWanted;
    private Timer m_stateChangedTimer;
    private boolean m_stateChanged;

    private boolean m_retractIntake;

    private final SendableChooser<LaunchStrategy> m_launchStrategyChooser;

    public Superstructure(
            final CommandSwerveDrivetrain swerve,
            final Launcher launcher,
            final Hopper hopper,
            final Uptake uptake,
            final Hood hood,
            final Intake intake
    ) {
        m_swerve = swerve;
        m_launcher = launcher;
        m_hopper = hopper;
        m_uptake = uptake;
        m_hood = hood;
        m_intake = intake;

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_wantedStatePublisher = myTable.getStringTopic("wanted_state").publish();
        m_systemStatePublisher = myTable.getStringTopic("system_state").publish();
        m_hubDistancePublisher = myTable.getDoubleTopic("target_distance_m").publish();
        m_launchVelocityPublisher = myTable.getDoubleTopic("launch_velocity_rps").publish();
        m_launchAnglePublisher = myTable.getStructTopic("launch_angle", Rotation2d.struct).publish();

        m_launchStrategyChooser = new SendableChooser<>();
        m_launchStrategyChooser.setDefaultOption("LookupTable", new LookupTableStrategy());
        m_launchStrategyChooser.addOption("SinMap", new SinMapStrategy());
        m_launchStrategyChooser.addOption("TunedLaunch", new TunedLaunchStrategy(myTable));
        SmartDashboard.putData("Launch Strategy", m_launchStrategyChooser);

        m_state = SystemState.IDLE;
        m_stateWanted = SystemState.IDLE;
        m_stateChangedTimer = new Timer();
        m_stateChanged = false;
    }

    @Override
    public void periodic() {
        m_systemStatePublisher.set(m_state.name());
        m_wantedStatePublisher.set(m_stateWanted.name());
        m_hubDistancePublisher.set(getDistanceToHub());
        final LaunchParameters parameters = getSelectedLaunchParameters();
        m_launchVelocityPublisher.set(parameters.rps());
        m_launchAnglePublisher.set(parameters.hoodAngle());

        // update subsystems periodically
        m_launcher.periodic();
        m_hopper.periodic();
        m_uptake.periodic();
        m_hood.periodic();

        final SystemState newState = switch (m_stateWanted) {
            case IDLE -> handleIdle();
            case LAUNCHING -> handleLaunching();
            case INTAKING -> handleIntaking();
            case CLIMBING -> handleClimbing();
            case CLIMBED -> handleClimbed();
            default -> SystemState.IDLE;
        };

        if (newState != m_state) {
            m_state = newState;
            m_stateChangedTimer.reset();
            m_stateChanged = true;
        } else {
            m_stateChanged = false;
        }
    }

    public SystemState handleIdle() {
        m_launcher.setOff();
        m_uptake.setIdle();
        m_intake.setStop();
        return SystemState.IDLE;
    }

    public SystemState handleLaunching() {
        conformToLaunchParameters(getSelectedLaunchParameters());

        if (m_launcher.getVelocity() > 30) {
            m_uptake.setVelocity(30);
        }

        return SystemState.LAUNCHING;
    }

    public SystemState handleIntaking() {
        m_intake.setIntaking();
        m_intake.setRetract();
        //m_intake.setDeploy();
        return SystemState.INTAKING;
    }

    public SystemState handleClimbing() {
        return SystemState.CLIMBING;
    }

    public SystemState handleClimbed() {
        return SystemState.CLIMBED;
    }

    public Command setWantedState(final SystemState state) {
        return runOnce(() -> {
            m_stateWanted = state;
        });
    }

    public Command applyWantedState(final SystemState newState) {
        return run(() -> {
            m_stateWanted = newState;
        }).until(() -> m_state == newState);
    }

    private double getDistanceToHub() {
        // get our hub
        final Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return target.getDistance(m_swerve.getState().Pose.getTranslation());
    }

    public LaunchParameters getSelectedLaunchParameters() {
        return m_launchStrategyChooser.getSelected().getParameters(getDistanceToHub());
    }

    private void conformToLaunchParameters(final LaunchParameters parameters) {
        m_hood.setReference(parameters.hoodAngle());
        m_launcher.setVelocity(parameters.rps());
    }

    public boolean isLaunching() {
        return m_launcher.getVelocity() > 33 && m_uptake.getVelocity() > 28 && m_hopper.getVelocity() > 28;
    }

    public void setRetractIntake(boolean retractIntake) {
        m_retractIntake = retractIntake;
    }

    public SystemState getState() {
        return m_state;
    }

    public Launcher getLauncher() {
        return m_launcher;
    }

    public Hood getHood() {
        return m_hood;
    }

    public Hopper getHopper() {
        return m_hopper;
    }
}
