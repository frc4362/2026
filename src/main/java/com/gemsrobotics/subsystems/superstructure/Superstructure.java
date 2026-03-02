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
    private final Launcher m_launcherEast, m_launcherWest;
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
            final Launcher launcherEast,
            final Launcher launcherWest,
            final Hopper hopper,
            final Uptake uptakeEast,
            final Hood hood,
            final Intake intake
    ) {
        m_swerve = swerve;
        m_launcherEast = launcherEast;
        m_launcherWest = launcherWest;
        m_hopper = hopper;
        m_uptake = uptakeEast;// uptake;
        m_hood = null;// hood;
        m_intake = intake;

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_wantedStatePublisher = myTable.getStringTopic("wanted_state").publish();
        m_systemStatePublisher = myTable.getStringTopic("system_state").publish();
        m_hubDistancePublisher = myTable.getDoubleTopic("target_distance_m").publish();
        m_launchVelocityPublisher = myTable.getDoubleTopic("launch_velocity_rps").publish();
        m_launchAnglePublisher = myTable.getStructTopic("launch_angle", Rotation2d.struct).publish();

        m_launchStrategyChooser = new SendableChooser<>();
        m_launchStrategyChooser.addOption("LookupTable", new LookupTableStrategy());
        m_launchStrategyChooser.addOption("SinMap", new SinMapStrategy());
        m_launchStrategyChooser.setDefaultOption("TunedLaunch", new TunedLaunchStrategy(myTable));
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
        final LauncherParameters parameters = getSelectedLaunchParameters();
        m_launchVelocityPublisher.set(parameters.rps());
        m_launchAnglePublisher.set(parameters.hoodAngle());

        // update subsystems periodically
//        m_launcher.periodic();
        m_hopper.periodic();
        m_uptake.periodic();
//        m_hood.periodic();

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
        m_intake.setStop();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
        return SystemState.IDLE;
    }

    private boolean m_isSpunUp = false;
    public SystemState handleLaunching() {
        if (m_stateChanged) {
            m_isSpunUp = false;
        }

        conformToLaunchParameters(getSelectedLaunchParameters());

        if (m_isSpunUp || (m_launcherEast.isAtReference() && m_launcherWest.isAtReference())) {
            m_intake.setRetractSlowly();
            m_uptake.setVoltage(11);
            m_hopper.setVelocity(66);
            m_isSpunUp = true;
        }

        return SystemState.LAUNCHING;
    }

    public SystemState handleIntaking() {
        m_intake.setIntaking();
        m_intake.setDeploy();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
        return SystemState.INTAKING;
    }

    public SystemState handleClimbing() {
        m_intake.setStop();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
        return SystemState.CLIMBING;
    }

    public SystemState handleClimbed() {
        m_intake.setStop();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
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

    public LauncherParameters getSelectedLaunchParameters() {
        return m_launchStrategyChooser.getSelected().getParameters(getDistanceToHub());
    }

    private void conformToLaunchParameters(final LauncherParameters parameters) {
//        m_hood.setReference(parameters.hoodAngle());
        m_launcherEast.setAngularVelocity(parameters.rps());
        m_launcherWest.setAngularVelocity(parameters.rps());
    }

    public boolean isLaunching() {
        return m_state == SystemState.LAUNCHING && m_isSpunUp;// m_launcher.getVelocity() > 33 && m_uptake.getVelocity() > 28 && m_hopper.getVelocity() > 28;
    }

    public void setRetractIntake(boolean retractIntake) {
        m_retractIntake = retractIntake;
    }

    public Rotation2d getIntakeAngle() {
        return m_intake.getAngle();
    }

    public Rotation2d getHoodAngle() {
        return Rotation2d.kZero;// m_hood.getLaunchAngle();
    }

    public SystemState getState() {
        return m_state;
    }
}
