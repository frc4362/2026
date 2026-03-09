package com.gemsrobotics;

import java.sql.Driver;
import java.util.Optional;
import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public final class MatchStateScheduler {
    private double m_timeLeftInState;
    private boolean m_isActive;
    private Optional<Boolean> m_wonAuto;

    private final Timer m_matchTimer;
    private boolean m_hasEverEnabledTeleop;

    private final SendableChooser<SchedulerMode> m_schedulerModeChooser;

    public MatchStateScheduler() {
        m_timeLeftInState = 9999;
        m_isActive = true;
        m_wonAuto = Optional.empty();

        m_schedulerModeChooser = new SendableChooser<>();
        m_schedulerModeChooser.setDefaultOption("FmsBased", SchedulerMode.FMS_BASED);
        m_schedulerModeChooser.addOption("AlwaysActive", SchedulerMode.ALWAYS_ACTIVE);
        m_schedulerModeChooser.addOption("RedWonAuto", SchedulerMode.RED_WON_AUTO);
        m_schedulerModeChooser.addOption("BlueWonAuto", SchedulerMode.BLUE_WON_AUTO);
        m_schedulerModeChooser.addOption("RandomizeOnEnable", SchedulerMode.RANDOMIZE_ON_ENABLE);
        m_schedulerModeChooser.onChange((unused) -> {
            m_wonAuto = Optional.empty(); // Allows changes after auto winner has been determined for the first time
        });
        SmartDashboard.putData("Match State Scheduler Mode", m_schedulerModeChooser);

        m_matchTimer = new Timer();
        m_matchTimer.start();
        m_hasEverEnabledTeleop = false;
        RobotModeTriggers.autonomous().onTrue(runOnce(m_matchTimer::restart));
        RobotModeTriggers.teleop().onTrue(runOnce(() -> {
            if (m_schedulerModeChooser.getSelected() == SchedulerMode.RANDOMIZE_ON_ENABLE) {
                m_wonAuto = Optional.empty();
            }
            if (!m_hasEverEnabledTeleop) {
                m_matchTimer.restart();
                m_hasEverEnabledTeleop = true;
            }
        }));
        RobotModeTriggers.test().onTrue(runOnce(m_matchTimer::restart));
    }

    public enum SchedulerMode {
        FMS_BASED,
        ALWAYS_ACTIVE,
        RED_WON_AUTO,
        BLUE_WON_AUTO,
        RANDOMIZE_ON_ENABLE
    }

    public record MatchState(double timeLeftInState, boolean isActive, Optional<Boolean> wonAuto) {
        public String toString() {
            String str = "";

            str += (isActive ? "active period, " : "inactive period, ");
            if (wonAuto.isPresent()) {
                str += (wonAuto.get() ? "won auto, " : "lost auto, ");
            } else {
                str += "no auto winner, ";
            }
            str += Math.round(timeLeftInState * 10.0) / 10.0 + "s until state change";

            return str;
        }
    }

    public void update() {
        // Unless inactive or in a timed period use these values
        m_timeLeftInState = 9999;
        m_isActive = true;

        if (m_schedulerModeChooser.getSelected() == SchedulerMode.ALWAYS_ACTIVE
                || DriverStation.getAlliance().isEmpty()) {
            return;
        }

        // Determine m_wonAuto
        if (m_wonAuto.isEmpty()) {
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
                if (m_wonAuto.isPresent()) {
                    m_timeLeftInState = (m_wonAuto.get() ? 10 : 35) - m_matchTimer.get();
                }
            } else if (m_matchTimer.get() < 35) {
                m_timeLeftInState = 35 - m_matchTimer.get();
                m_isActive = !m_wonAuto.get();
            } else if (m_matchTimer.get() < 60) {
                m_timeLeftInState = 60 - m_matchTimer.get();
                m_isActive = m_wonAuto.get();
            } else if (m_matchTimer.get() < 85) {
                m_timeLeftInState = 85 - m_matchTimer.get();
                m_isActive = !m_wonAuto.get();
            } else if (m_matchTimer.get() < 110) {
                m_isActive = m_wonAuto.get();
                if (!m_isActive) {
                    m_timeLeftInState = 110 - m_matchTimer.get();
                }
            }
        }
    }

    private void determineIfWonAuto(Alliance winningAlliance) {
        Alliance teamAlliance = DriverStation.getAlliance().get();
        m_wonAuto = (teamAlliance == winningAlliance) ? Optional.of(true) : Optional.of(false);
    }

    public MatchState getMatchState() {
        return new MatchState(m_timeLeftInState, m_isActive, m_wonAuto);
    }
}