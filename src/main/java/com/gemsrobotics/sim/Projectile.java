package com.gemsrobotics.sim;

import com.gemsrobotics.FieldConstants;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

public class Projectile {
    private static final boolean BOUNCE = true;
    private static final double FUEL_RADIUS = 0.075;

    private Pose3d pose;
    private Translation3d velocity;
    private double lastTimeStamp;

    public Projectile(final Pose3d startingPose, final Translation3d startingVelocity) {
        pose = startingPose;
        velocity = startingVelocity;

        lastTimeStamp = -1;
    }

    public void update() {
        if (lastTimeStamp == -1) {
            lastTimeStamp = Timer.getTimestamp();
        }

        double timestamp = Timer.getTimestamp();
        double dt = timestamp - lastTimeStamp;

        pose = new Pose3d(pose.getX() + velocity.getX() * dt, pose.getY() + velocity.getY() * dt, pose.getZ() + velocity.getZ() * dt, pose.getRotation());
        velocity = velocity.plus(new Translation3d(0.0, 0.0, -9.8 * dt));



        // Bouncing

        if(BOUNCE) {
            if (pose.getZ() < FUEL_RADIUS / 2 && velocity.getZ() < 0) {
                velocity = new Translation3d(velocity.getX(), velocity.getY(), velocity.getZ() * -0.8);
            }

            // X walls
            if ((pose.getX() < FUEL_RADIUS && velocity.getX() < 0) || (pose.getX() > FieldConstants.fieldLength - FUEL_RADIUS && velocity.getX() > 0)) {
                velocity = new Translation3d(velocity.getX() * -0.8, velocity.getY(), velocity.getZ());
            }

            // Y walls
            if ((pose.getY() < FUEL_RADIUS && velocity.getY() < 0) || (pose.getY() > FieldConstants.fieldWidth - FUEL_RADIUS && velocity.getY() > 0)) {
                velocity = new Translation3d(velocity.getX(), velocity.getY() * -0.8, velocity.getZ());
            }
        }

        lastTimeStamp = timestamp;
    }

    public Pose3d getPose() {
        return pose;
    }

    public boolean isDone() {
        return Math.abs(velocity.getZ()) < 0.1 && Math.abs(pose.getZ()) < 0.1;
    }
}