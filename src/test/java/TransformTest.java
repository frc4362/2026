import com.gemsrobotics.Constants;
import com.gemsrobotics.commands.PilotedDrive;
import com.gemsrobotics.lib.math.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import jdk.dynalink.support.ChainedCallSite;
import org.junit.jupiter.api.Test;

import static java.lang.Math.hypot;
import static java.lang.Math.sqrt;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class TransformTest {
    @Test
    public void testLauncherTransform() {
        final Transform2d VEHICLE_TO_LAUNCHER = new Transform2d(-2, -2, Rotation2d.fromDegrees(180));
        final Pose2d vehiclePose = new Pose2d(2, 2, Rotation2d.fromDegrees(0.0));
        System.out.println(vehiclePose.transformBy(VEHICLE_TO_LAUNCHER));
    }

    private static double magnitude(final ChassisSpeeds chassisSpeeds) {
        return hypot(chassisSpeeds.vxMetersPerSecond,  chassisSpeeds.vyMetersPerSecond);
    }

    private static ChassisSpeeds scaleChassisSpeeds(final ChassisSpeeds desiredVelocity, final double speed) {
        final double currentChassisSpeeds = hypot(desiredVelocity.vxMetersPerSecond, desiredVelocity.vyMetersPerSecond);
        final double scalar = speed / currentChassisSpeeds;
        return new ChassisSpeeds(
                desiredVelocity.vxMetersPerSecond * scalar,
                desiredVelocity.vyMetersPerSecond * scalar,
                desiredVelocity.omegaRadiansPerSecond * scalar);
    }

    @Test
    public void testLimitIntakeVelocity() {
        final double MAX_VELOCITY = 2.75;
        final Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kZero);
        final ChassisSpeeds desiredVelocity = new ChassisSpeeds(3.0, 0.0, 3);
        final Transform2d intakeCornerPose = Constants.INTAKE_CORNER_NE;
        System.out.println("vehicle speeds = " + desiredVelocity);
        System.out.println("magnitude = " + magnitude(desiredVelocity));
        final var intakeSpeeds = GeometryUtil.transformVelocity(desiredVelocity, intakeCornerPose, robotPose.getRotation());
        System.out.println("intake speeds = " + intakeSpeeds);
        System.out.println("magnitude = " + magnitude(intakeSpeeds));
        final var scaledIntakeSpeeds = scaleChassisSpeeds(intakeSpeeds, MAX_VELOCITY);
        System.out.println("new intake speeds = " + scaledIntakeSpeeds);
        System.out.println("magnitude = " + magnitude(scaledIntakeSpeeds));
        final var newBodySpeeds = GeometryUtil.transformVelocity(scaledIntakeSpeeds, intakeCornerPose.inverse(), robotPose.getRotation());
        System.out.println("new vehicle speeds = " + newBodySpeeds);
        System.out.println("magnitude = " + magnitude(newBodySpeeds));
    }

    @Test
    public void testTurnVelocity() {
        final ChassisSpeeds desiredVelocity = new ChassisSpeeds(1.5, 0.0, Math.PI);
        final Transform2d intakeCornerPose = Constants.INTAKE_CORNER_NE;
        final var intakeSpeeds = GeometryUtil.transformVelocity(desiredVelocity, intakeCornerPose, Rotation2d.kZero);
        System.out.println("intake speeds = " + intakeSpeeds);
        System.out.println("magnitude = " + magnitude(intakeSpeeds));
    }

    @Test
    public void testLimitSpeedsForIntaking() {
        final ChassisSpeeds desiredVelocity = new ChassisSpeeds(1.5, 0.0, Math.PI);
        final Transform2d intakeCornerPose = Constants.INTAKE_CORNER_NE;
        final ChassisSpeeds limitedSpeeds = PilotedDrive.limitSpeedsForIntaking(desiredVelocity, Rotation2d.kZero);
        System.out.println("desired velocity = " + desiredVelocity);
        System.out.println("desired velocity intake speed = " + GeometryUtil.transformVelocity(desiredVelocity, intakeCornerPose, Rotation2d.kZero));
        System.out.println("desired velocity intake speed magnitude = " + magnitude(GeometryUtil.transformVelocity(desiredVelocity, intakeCornerPose, Rotation2d.kZero)));
        System.out.println("limited velocity = " + limitedSpeeds);
        System.out.println("limited velocity magnitude = " + magnitude(limitedSpeeds));
        System.out.println("limited velocity intake speed = " + GeometryUtil.transformVelocity(limitedSpeeds, intakeCornerPose, Rotation2d.kZero));
        System.out.println("limited velocity intake speed magnitude = " + magnitude(GeometryUtil.transformVelocity(limitedSpeeds, intakeCornerPose, Rotation2d.kZero)));
    }
}
