package com.gemsrobotics;

import java.util.Random;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public final class MatchStateScheduler {

    public static final int UNLIMITED_TIME_IN_STATE = 5940;

    public enum WonAuto implements StructSerializable {
        TRUE,
        FALSE,
        INDETERMINATE;

        public static final Struct<WonAuto> struct = StructGenerator.genEnum(WonAuto.class);
    }

    private double m_timeLeftInState;
    private boolean m_isActive;
    private WonAuto m_wonAuto;

    private final Timer m_matchTimer;
    private boolean m_hasEverEnabledTeleop;

    private final SendableChooser<SchedulerMode> m_schedulerModeChooser;

    private final NetworkTable m_table;
    private final DoublePublisher m_timeLeftPublisher;
    private final BooleanPublisher m_activePublisher;
    private final StringPublisher m_wonAutoPublisher;

    public MatchStateScheduler() {
        m_timeLeftInState = UNLIMITED_TIME_IN_STATE;
        m_isActive = true;
        m_wonAuto = WonAuto.INDETERMINATE;

        // Defaults to AlwaysActive so in-match disconnects do not cause timing problems
        m_schedulerModeChooser = new SendableChooser<>();
        m_schedulerModeChooser.addOption("FmsBased", SchedulerMode.FMS_BASED);
        m_schedulerModeChooser.setDefaultOption("AlwaysActive", SchedulerMode.ALWAYS_ACTIVE);
        m_schedulerModeChooser.addOption("RedWonAuto", SchedulerMode.RED_WON_AUTO);
        m_schedulerModeChooser.addOption("BlueWonAuto", SchedulerMode.BLUE_WON_AUTO);
        m_schedulerModeChooser.addOption("RandomizeOnEnable", SchedulerMode.RANDOMIZE_ON_ENABLE);
        m_schedulerModeChooser.onChange((unused) -> {
            m_wonAuto = WonAuto.INDETERMINATE; // Allows changes after auto winner has been determined for the first time
        });
        SmartDashboard.putData("Match State Scheduler Mode", m_schedulerModeChooser);

        m_matchTimer = new Timer();
        m_matchTimer.start();
        m_hasEverEnabledTeleop = false;
        RobotModeTriggers.autonomous().onTrue(runOnce(m_matchTimer::restart));
        RobotModeTriggers.teleop().onTrue(runOnce(() -> {
            if (m_schedulerModeChooser.getSelected() == SchedulerMode.RANDOMIZE_ON_ENABLE) {
                m_wonAuto = WonAuto.INDETERMINATE;
            }
            if (!m_hasEverEnabledTeleop) {
                m_matchTimer.restart();
                m_hasEverEnabledTeleop = true;
            }
        }));
        RobotModeTriggers.test().onTrue(runOnce(m_matchTimer::restart));

        m_table = NetworkTableInstance.getDefault().getTable("match-state-scheduler");
        m_timeLeftPublisher = m_table.getDoubleTopic("time_left_in_state").publish();
        m_activePublisher = m_table.getBooleanTopic("is_active").publish();
        m_wonAutoPublisher = m_table.getStringTopic("won_auto").publish();
    }

    public enum SchedulerMode {
        FMS_BASED,
        ALWAYS_ACTIVE,
        RED_WON_AUTO,
        BLUE_WON_AUTO,
        RANDOMIZE_ON_ENABLE
    }

    public record MatchState(double timeLeftInState, boolean isActive, WonAuto wonAuto) {
        public double getTimeUntilActive() {
            return isActive ? 0.0 : timeLeftInState;
        }
    }

    public void update() {
        // Unless inactive or in a timed period use these values
        m_timeLeftInState = UNLIMITED_TIME_IN_STATE;
        m_isActive = true;

        if (m_schedulerModeChooser.getSelected() == SchedulerMode.ALWAYS_ACTIVE
                || DriverStation.getAlliance().isEmpty()) {
            return;
        }

        // Determine m_wonAuto
        if (m_wonAuto == WonAuto.INDETERMINATE) {
            switch (m_schedulerModeChooser.getSelected()) {
                case FMS_BASED -> {
                    String gameData = DriverStation.getGameSpecificMessage();

                    if (!gameData.isEmpty()) {
                        switch(gameData.charAt(0)) {
                            case 'B':
                                determineIfWonAuto(Alliance.Blue);
                                break;
                            case 'R':
                                determineIfWonAuto(Alliance.Red);
                                break;
                            default:
                                break;
                        }
                    } else if (DriverStation.isTeleop() && m_matchTimer.get() >= 10) {
                        return;
                    }
                }
                case RED_WON_AUTO -> determineIfWonAuto(Alliance.Red);
                case BLUE_WON_AUTO -> determineIfWonAuto(Alliance.Blue);
                case RANDOMIZE_ON_ENABLE -> {
                    Random rand = new Random();
                    determineIfWonAuto(rand.nextBoolean() ? Alliance.Blue : Alliance.Red);
                }
            }
        }

        // Just in case
        if (DriverStation.isDisabled()) return;

        // Override defaults if inactive or in a timed period
        if (DriverStation.isAutonomous()) {
            m_timeLeftInState = 20 - m_matchTimer.get();
        } else if (DriverStation.isTeleop()) {
            if (m_matchTimer.get() < 10) {
                if (m_wonAuto != WonAuto.INDETERMINATE) {
                    m_timeLeftInState = (m_wonAuto == WonAuto.TRUE ? 10 : 35) - m_matchTimer.get();
                }
            } else if (m_matchTimer.get() < 35) {
                m_timeLeftInState = 35 - m_matchTimer.get();
                m_isActive = m_wonAuto == WonAuto.FALSE;
            } else if (m_matchTimer.get() < 60) {
                m_timeLeftInState = 60 - m_matchTimer.get();
                m_isActive = m_wonAuto == WonAuto.TRUE;
            } else if (m_matchTimer.get() < 85) {
                m_timeLeftInState = 85 - m_matchTimer.get();
                m_isActive = m_wonAuto == WonAuto.FALSE;
            } else if (m_matchTimer.get() < 110) {
                m_timeLeftInState = (m_wonAuto == WonAuto.TRUE ? 140 : 110) - m_matchTimer.get();
                m_isActive = m_wonAuto == WonAuto.TRUE;
            } else if (m_matchTimer.get() < 140) {
                m_timeLeftInState = 140 - m_matchTimer.get();
            }
        }
    }

    private void determineIfWonAuto(Alliance winningAlliance) {
        Alliance teamAlliance = DriverStation.getAlliance().get();
        m_wonAuto = (teamAlliance == winningAlliance) ? WonAuto.TRUE : WonAuto.FALSE;
    }

    public MatchState getMatchState() {
        return new MatchState(m_timeLeftInState, m_isActive, m_wonAuto);
    }

    public void logMatchState() {
        m_timeLeftPublisher.set(m_timeLeftInState);
        m_activePublisher.set(m_isActive);

        String wonAutoColors = switch (m_wonAuto) {
            case TRUE -> "#4CAF50";
            case FALSE -> "#F44336";
            case INDETERMINATE -> "#888888";
        };

        m_wonAutoPublisher.set(wonAutoColors);
    }
}