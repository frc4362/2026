package com.gemsrobotics.sim;

import com.gemsrobotics.Constants;
import com.gemsrobotics.RobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ProjectileManager {
    private static final boolean DO_SHOT_VARIANCE = true;
    private static final double SHOT_VARIANCE_DEGREES = 1.0;
    private static final double SHOT_COOLDOWN = 0.25;
    private static final Transform3d ROBOT_TO_LAUNCHER = new Transform3d(Constants.ROBOT_TO_LAUNCHER)
            .plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));

    private final RobotState m_robotState;
    private final List<Projectile> m_projectiles;
    private final StructArrayPublisher<Pose3d> m_fuelPublisher;
    private final Supplier<LinearVelocity> m_launchVelocity;
    private final Supplier<Rotation2d> m_launchPitch;

    private double m_lastLaunchTime;

    public ProjectileManager(
            final RobotState robotState,
            final Supplier<LinearVelocity> velocitySupplier,
            final Supplier<Rotation2d> releasePitchSupplier
    ) {
        m_robotState = robotState;
        m_projectiles = new ArrayList<>();

        final NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.SIM_VIZ_TABLE_KEY);
        m_fuelPublisher = table.getStructArrayTopic("fuel", Pose3d.struct).publish();

        m_launchVelocity = velocitySupplier;
        m_launchPitch = releasePitchSupplier;

        m_lastLaunchTime = 0.0;
    }

    public void attemptSpawn() {
        final double currentTime = Timer.getTimestamp();
        if ((currentTime - m_lastLaunchTime) < SHOT_COOLDOWN) {
            return;
        }
        m_lastLaunchTime = currentTime;

        final var robotPose = m_robotState.getLatestFieldToVehicle().getValue();

        final double velocity = m_launchVelocity.get().in(Units.MetersPerSecond);
        // start at the robot's pose plus the launcher's pose
        final var startingPose = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians())).plus(ROBOT_TO_LAUNCHER);

        final var impartedRobotVelocity = new Translation3d(
                m_robotState.getLatestChassisSpeeds_FieldRelative().vxMetersPerSecond,
                m_robotState.getLatestChassisSpeeds_FieldRelative().vyMetersPerSecond,
                0);
        // our shot is forward and up by its angular components, aimed by robot heading

        var launchPitch = m_launchPitch.get();
        if (DO_SHOT_VARIANCE) {
            launchPitch = launchPitch.plus(Rotation2d.fromDegrees(SHOT_VARIANCE_DEGREES * 2 * (Math.random() - 0.5)));
        }

        final var shotVelocity = new Translation3d(launchPitch.getCos() * velocity, 0, launchPitch.getSin() * velocity)
                .rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        final var startingVelocity = impartedRobotVelocity.plus(shotVelocity);

        m_projectiles.add(new Projectile(startingPose, startingVelocity));
    }

    public void updateAll() {
        final List<Pose3d> currentPoses = new ArrayList<>();

        for (int i = m_projectiles.size() - 1; i >= 0; i--) {
            final var projectile = m_projectiles.get(i);

            if (projectile.isDone()) {
                m_projectiles.remove(i);
            } else {
                projectile.update();
                currentPoses.add(projectile.getPose());
            }
        }

        m_fuelPublisher.set(currentPoses.toArray(new Pose3d[0]));
    }
}
