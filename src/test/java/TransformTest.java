import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TransformTest {
    @Test
    public void testLauncherTransform() {
        final Transform2d VEHICLE_TO_LAUNCHER = new Transform2d(-2, -2, Rotation2d.fromDegrees(180));
        final Pose2d vehiclePose = new Pose2d(2, 2, Rotation2d.fromDegrees(0.0));
        System.out.print(vehiclePose.transformBy(VEHICLE_TO_LAUNCHER));
    }
}
