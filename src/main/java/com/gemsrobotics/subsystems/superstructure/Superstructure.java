package com.gemsrobotics.subsystems.superstructure;

import com.ctre.phoenix6.Orchestra;
import com.gemsrobotics.Constants;
import com.gemsrobotics.FieldConstants;
import com.gemsrobotics.RobotState;
import com.gemsrobotics.launching.*;
import com.gemsrobotics.subsystems.swerve.CommandSwerveDrivetrain;
import com.gemsrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public final class Superstructure extends SubsystemBase {
    private static final String NT_KEY = "superstructure";

    public enum SystemState {
        IDLE,
        LAUNCHING,
        INTAKING,
        SPITTING
    }

    public static final boolean DO_INTAKE_AGITATION = true;
    public static final double INTAKE_AGITATION_DELAY = 1.75;
    public static final double INTAKE_AGITATION_FREQUENCY = 2.0;
    public static final double INTAKE_AGITATION_PHASE = 1.0 / INTAKE_AGITATION_FREQUENCY;

    private final CommandSwerveDrivetrain m_swerve;
    private final Launcher m_launcherEast, m_launcherWest;
    private final Hopper m_hopper;
    private final Uptake m_uptake;
    private final Hood m_hood;
    private final Intake m_intake;
    private final RobotState m_robotState;

    private final Orchestra m_orchestra;
    private final String CHRP_FILENAME = "MoonlightSonata.chrp";

    private final StringPublisher m_systemStatePublisher;
    private final StringPublisher m_wantedStatePublisher;
    private final DoublePublisher m_hubDistancePublisher, m_launchVelocityPublisher, m_stateChangeTimePublisher;
    private final StructPublisher<Rotation2d> m_launchAnglePublisher;
    private final BooleanPublisher m_isLaunchingPublisher;

    private SystemState m_state;
    private SystemState m_stateWanted;
    private Timer m_stateChangedTimer;
    private boolean m_stateChanged;

    private boolean m_isAllowedToLaunch;
    private LaunchingCalculator.Parameters m_launcherParameters;
    private boolean m_hasEverDeployedIntake;
    private boolean m_retractIntake;
    private boolean m_doEarlyAgitation;
    private boolean m_wantsIntaking;

    private TunedLaunchStrategy m_tunedLaunchStrategy;

    private final SendableChooser<Boolean> m_doTuningChooser;

    public Superstructure(
            final CommandSwerveDrivetrain swerve,
            final Launcher launcherEast,
            final Launcher launcherWest,
            final Hopper hopper,
            final Uptake uptakeEast,
            final Hood hood,
            final Intake intake,
            final RobotState robotState
    ) {
        m_swerve = swerve;
        m_launcherEast = launcherEast;
        m_launcherWest = launcherWest;
        m_hopper = hopper;
        m_uptake = uptakeEast;// uptake;
        m_hood =  hood;
        m_intake = intake;
        m_robotState = robotState;

        //region Orchestra tomfoolery
        m_orchestra = new Orchestra();
        m_orchestra.addInstrument(m_uptake.getLeaderMotor());
        m_orchestra.addInstrument(m_uptake.getFollowerMotor());
        m_orchestra.loadMusic(CHRP_FILENAME);
        //endregion

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable(NT_KEY);
        m_wantedStatePublisher = myTable.getStringTopic("wanted_state").publish();
        m_systemStatePublisher = myTable.getStringTopic("system_state").publish();
        m_hubDistancePublisher = myTable.getDoubleTopic("target_distance_m").publish();
        m_stateChangeTimePublisher = myTable.getDoubleTopic("state_change_time").publish();
        m_launchVelocityPublisher = myTable.getDoubleTopic("launch_velocity_rps").publish();
        m_launchAnglePublisher = myTable.getStructTopic("launch_angle", Rotation2d.struct).publish();
        m_isLaunchingPublisher = myTable.getBooleanTopic("is_launching").publish();

        m_tunedLaunchStrategy = new TunedLaunchStrategy(myTable);

        m_doTuningChooser = new SendableChooser<>();
        m_doTuningChooser.addOption("Tuning", true);
        m_doTuningChooser.setDefaultOption("Pre-calibrated", false);
        SmartDashboard.putData("Do Tuning", m_doTuningChooser);

        m_state = SystemState.IDLE;
        m_stateWanted = SystemState.IDLE;
        m_stateChangedTimer = new Timer();
        m_stateChanged = false;

        m_isAllowedToLaunch = true;

        m_wantsIntaking = false;
        m_hasEverDeployedIntake = false;
        m_doEarlyAgitation = false;
    }

    @Override
    public void periodic() {
        m_systemStatePublisher.set(m_state.name());
        m_wantedStatePublisher.set(m_stateWanted.name());
        m_hubDistancePublisher.set(getDistanceToHub());
        m_stateChangeTimePublisher.set(m_stateChangedTimer.get());
        m_isLaunchingPublisher.set(isLaunching());

        getSelectedLaunchParameters().ifPresent(parameters -> {
            m_launchVelocityPublisher.set(parameters.getRps());
            m_launchAnglePublisher.set(parameters.getHoodAngle());
        });

        // update subsystems periodically
        m_hood.periodic();

        final SystemState newState = switch (m_state) {
            case IDLE -> handleIdle();
            case LAUNCHING -> handleLaunching();
            case INTAKING -> handleIntaking();
            case SPITTING -> handleSpitting();
            default -> SystemState.IDLE;
        };

        if (newState != m_state) {
            m_state = newState;
            m_stateChangedTimer.restart();
            m_stateChanged = true;
        } else {
            m_stateChanged = false;
        }

        if (m_orchestra.isPlaying() && DriverStation.isEnabled()) {
            m_orchestra.stop();
        }
    }

    public SystemState conformToWantedState() {
        // this could contain more complex state-change logic later on
        return m_stateWanted;
    }

    public SystemState handleIdle() {
//        if (DriverStation.isDisabled() && m_robotState.getLastVisionPoseEstimate().tagCount() > 1) {
//            m_orchestra.play();
//        }

        if (m_retractIntake) {
            m_intake.setRetract();
        } else if (m_hasEverDeployedIntake) {
            m_intake.setDeploy();
        }

        m_intake.setStop();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIdle();
        m_hopper.setIdle();
        return conformToWantedState();
    }

    private final Timer m_intakeLiftTimer = new Timer();
    private boolean m_isSpunUp = false;
    private double m_startAgitationTimestamp = -1;
    public SystemState handleLaunching() {
        if (m_stateChanged) {
            if (m_intakeLiftTimer.isRunning()) {
                m_intakeLiftTimer.stop();
            }
            m_intakeLiftTimer.reset();
            m_isSpunUp = false;
            m_startAgitationTimestamp = -1;
        }

        getSelectedLaunchParameters().ifPresent(this::conformToLaunchParameters);

        if ((m_isSpunUp || isReadyToStartLaunching()) && (m_isAllowedToLaunch || m_doTuningChooser.getSelected())) {
            if (!m_isSpunUp) {
                m_intakeLiftTimer.start();
            }

            final boolean doPush;
            final double s;
            if (m_doEarlyAgitation || (m_intakeLiftTimer.get() > INTAKE_AGITATION_DELAY)) {
                if (m_startAgitationTimestamp == -1) {
                    m_startAgitationTimestamp = m_intakeLiftTimer.get();
                }

                doPush = true;
                s = m_intakeLiftTimer.get() - m_startAgitationTimestamp;
            } else {
                doPush = false;
                s = m_stateChangedTimer.get();
            }

            if (m_wantsIntaking) {
                m_intake.setIntaking();
            } else if ((s % INTAKE_AGITATION_PHASE) < (INTAKE_AGITATION_PHASE / 2.0)) {
                if (doPush) {
                    m_intake.setAgitating();
                } else {
                    m_intake.setDeploy();
                }
//            } else if (m_intakeLiftTimer.get() > INTAKE_AGITATION_DELAY) {
//                m_intake.setRetractSlowly();
            } else {
                m_intake.setDeploy();
            }

            m_intake.setFeedingHopper();
            m_uptake.setFeeding();
            m_hopper.setFeeding();

            m_isSpunUp = true;
        } else {
            m_hopper.setIdle();
        }

        return conformToWantedState();
    }

    public SystemState handleIntaking() {
        m_intake.setDeploy();
        m_intake.setIntaking();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setIntaking();
        m_hopper.setIntaking();

        m_hasEverDeployedIntake = true;

        return conformToWantedState();
    }

    public SystemState handleSpitting() {
        m_intake.setSpitting();
        m_intake.setDeploy();
        m_launcherEast.setOff();
        m_launcherWest.setOff();
        m_uptake.setReversing();
        m_hopper.setSpitting();

        m_hasEverDeployedIntake = true;

        return conformToWantedState();
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

    public boolean isLaunching() {
        return m_hopper.isHopping.getAsBoolean() && m_state == SystemState.LAUNCHING && m_isSpunUp;
    }

    private double getDistanceToHub() {
        // get our hub
        final Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        return target.getDistance(m_swerve.getState().Pose.transformBy(Constants.ROBOT_TO_LAUNCHER).getTranslation());
    }

    public Optional<HoodAndRps> getSelectedLaunchParameters() {
        // never use tuning cals if we're attached to the FMS
        if (m_doTuningChooser.getSelected() && !DriverStation.isFMSAttached()) {
            return Optional.ofNullable(m_tunedLaunchStrategy.getParameters(getDistanceToHub()));
        } else {
            return Optional.ofNullable(m_launcherParameters);
        }
    }

    public boolean isReadyToStartLaunching() {
        return (m_isSpunUp || (m_launcherWest.atReference() && m_launcherEast.atReference())) && m_hood.atReference();
    }

    private void conformToLaunchParameters(final HoodAndRps parameters) {
        m_hood.setReference(parameters.getHoodAngle());
        m_launcherEast.setAngularVelocity(parameters.getRps());
        m_launcherWest.setAngularVelocity(parameters.getRps());
    }

    public void setRetractIntake(boolean retractIntake) {
        m_retractIntake = retractIntake;
    }

    public Rotation2d getIntakeAngle() {
        return m_intake.getAngle();
    }

    public Rotation2d getHoodAngle() {
        return m_hood.getLaunchAngle();
    }

    public SystemState getState() {
        return m_state;
    }

    public Intake getIntake() {
        return m_intake;
    }

    public void setLauncherParameters(final LaunchingCalculator.Parameters parameters) {
        m_launcherParameters = parameters;
    }

    public void setAllowedToLaunch(final boolean allowed) {
        m_isAllowedToLaunch = allowed;
    }

    public void setDoEarlyAgitation(final boolean doAgitation) {
        m_doEarlyAgitation = doAgitation;
    }

    public void setWantsIntaking(final boolean wantsIntaking) {
        m_wantsIntaking = wantsIntaking;
    }
}
