package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchStateTracker {

    private MatchPeriod m_period;
    private double m_timeLeft;
    private double m_timeUntilActive;
    private boolean m_isActive;
    private Optional<Boolean> m_wonAuto;

    private double m_matchTime;

    private final StringPublisher matchStatePublisher;

    public MatchStateTracker() {
        m_period = MatchPeriod.AUTO;
        m_timeLeft = 20;
        m_timeUntilActive = 0;
        m_isActive = true;
        m_wonAuto = Optional.empty();

        final NetworkTable myTable = NetworkTableInstance.getDefault().getTable("matchStateTracker");
        matchStatePublisher = myTable.getStringTopic("match state").publish();
    }

    enum MatchPeriod {
        AUTO,
        TRANSITION,
        PHASE1,
        PHASE2,
        PHASE3,
        PHASE4,
        ENDGAME,
        DEBUG
    }

    record MatchState(MatchPeriod period, double timeLeft, double timeUntilActive, boolean isActive) {
        public String toString() {
            if (isActive) {
                return "active " + period + " period, " + timeLeft + "s left";
            } else {
                return "inactive " + period + " period, " + timeLeft + "s left";
            }
        }
    }

    public void update() {
        m_matchTime = DriverStation.getMatchTime();
        // if FMS or practice match time counts down
        if (DriverStation.isFMSAttached() || DriverStation.getMatchTime() != -1) {
            if (m_wonAuto.isEmpty()) {
                String gameData = DriverStation.getGameSpecificMessage();

                if (m_matchTime < 135) m_wonAuto = Optional.of(true);

                if (gameData.length() > 0) {
                    Alliance alliance = DriverStation.getAlliance().get();
                    switch(gameData.charAt(0)) {
                        case 'B':
                            m_wonAuto = (alliance == Alliance.Blue) ? Optional.of(true) : Optional.of(false);
                            break;
                        case 'R':
                            m_wonAuto = (alliance == Alliance.Red) ? Optional.of(true) : Optional.of(false);
                            break;
                        default:
                            break;
                    }
                }
            }

            if (DriverStation.isAutonomous()) {
                m_period = MatchPeriod.AUTO;
                m_timeLeft = m_matchTime;
                m_timeUntilActive = 0;
            } else if (DriverStation.isTeleop()) {
                if (m_matchTime > 130) {
                    m_period = MatchPeriod.TRANSITION;
                    m_timeLeft = m_matchTime - 130;
                    m_timeUntilActive = 0;
                } else if (m_matchTime > 105) {
                    m_period = MatchPeriod.PHASE1;
                    m_timeLeft = m_matchTime - 105;
                    m_timeUntilActive = m_wonAuto.get() ? m_timeLeft : 0;
                } else if (m_matchTime > 80) {
                    m_period = MatchPeriod.PHASE2;
                    m_timeLeft = m_matchTime - 80;
                    m_timeUntilActive = !m_wonAuto.get() ? m_timeLeft : 0;
                } else if (m_matchTime > 55) {
                    m_period = MatchPeriod.PHASE3;
                    m_timeLeft = m_matchTime - 55;
                    m_timeUntilActive = m_wonAuto.get() ? m_timeLeft : 0;
                } else if (m_matchTime > 30) {
                    m_period = MatchPeriod.PHASE4;
                    m_timeLeft = m_matchTime - 30;
                    m_timeUntilActive = !m_wonAuto.get() ? m_timeLeft : 0;
                } else { // should go until 140 seconds
                    m_period = MatchPeriod.ENDGAME;
                    m_timeLeft = m_matchTime;
                    m_timeUntilActive = 0;
                }
            }

            if (m_timeLeft < 0) m_timeLeft = 0; // Avoid negative time left

            m_isActive = m_timeUntilActive == 0;
        } else { // if just practicing in a single mode
            m_period = MatchPeriod.DEBUG;
            m_timeLeft = 9999;
            m_timeUntilActive = 0;
            m_isActive = true;
        }
    }

    public MatchState getMatchState() {
        MatchState state = new MatchState(m_period, m_timeLeft, m_timeUntilActive, m_isActive);
        // matchStatePublisher.set(state.toString());
        return state;
    }
}
