package com.gemsrobotics.sim;

import com.gemsrobotics.RobotContainer;
import com.gemsrobotics.RobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Projectile {
    private static final boolean BOUNCE = true;

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

        // Floor
        if(BOUNCE && pose.getZ() < 0 && velocity.getZ() < 0) {
            velocity = new Translation3d(velocity.getX(), velocity.getY(), velocity.getZ() * -0.8);
        }

        // X walls
        if(BOUNCE && ((pose.getX() < 0 && velocity.getX() < 0) || (pose.getX() > 17.5 && velocity.getX() > 0))) {
            velocity = new Translation3d(velocity.getX() * -0.8, velocity.getY(), velocity.getZ());
        }

        // Y walls
        if(BOUNCE && ((pose.getY() < 0 && velocity.getY() < 0) || (pose.getY() > 8 && velocity.getY() > 0))) {
            velocity = new Translation3d(velocity.getX(), velocity.getY() * -0.8, velocity.getZ());
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