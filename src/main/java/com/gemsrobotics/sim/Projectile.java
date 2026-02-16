package com.gemsrobotics.sim;

import com.gemsrobotics.FieldConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class Projectile {
    private static final boolean DO_BOUNCE = true;
    private static final double FUEL_RADIUS = Inches.of(5.91 / 2.0).in(Meters);

    private Pose3d m_pose;
    private Translation3d m_velocity;
    private double m_lastTimestamp;

    public Projectile(final Pose3d startingPose, final Translation3d startingVelocity) {
        m_pose = startingPose;
        m_velocity = startingVelocity;

        m_lastTimestamp = -1;
    }

    public void update() {
        if (m_lastTimestamp == -1) {
            m_lastTimestamp = Timer.getTimestamp();
        }

        double timestamp = Timer.getTimestamp();
        double dt = timestamp - m_lastTimestamp;

        m_pose = new Pose3d(m_pose.getX() + m_velocity.getX() * dt, m_pose.getY() + m_velocity.getY() * dt, m_pose.getZ() + m_velocity.getZ() * dt, m_pose.getRotation());
        m_velocity = m_velocity.plus(new Translation3d(0.0, 0.0, -9.8 * dt));

        // Bouncing

        if (DO_BOUNCE) {
            if (m_pose.getZ() < FUEL_RADIUS / 2 && m_velocity.getZ() < 0) {
                m_velocity = new Translation3d(m_velocity.getX(), m_velocity.getY(), m_velocity.getZ() * -0.8);
            }

            // X walls
            if ((m_pose.getX() < FUEL_RADIUS && m_velocity.getX() < 0) || (m_pose.getX() > FieldConstants.fieldLength - FUEL_RADIUS && m_velocity.getX() > 0)) {
                m_velocity = new Translation3d(m_velocity.getX() * -0.8, m_velocity.getY(), m_velocity.getZ());
            }

            // Y walls
            if ((m_pose.getY() < FUEL_RADIUS && m_velocity.getY() < 0) || (m_pose.getY() > FieldConstants.fieldWidth - FUEL_RADIUS && m_velocity.getY() > 0)) {
                m_velocity = new Translation3d(m_velocity.getX(), m_velocity.getY() * -0.8, m_velocity.getZ());
            }
        }

        m_lastTimestamp = timestamp;
    }

    public Pose3d getPose() {
        return m_pose;
    }

    public boolean isDone() {
        return Math.abs(m_velocity.getZ()) < 0.1 && Math.abs(m_pose.getZ()) < 0.1;
    }
}