package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final double xyStdDevCoefficient = 0.04;
  public static final double thetaStdDevCoefficient = 0.08;

  public static Transform3d robotToCamArducamLeft = new Transform3d(new Translation3d(0.11, -0.2707, 0.2223),
      new Rotation3d(0, Units.degreesToRadians(-18.234),
          Units.degreesToRadians(7.2367)));

  public static Transform3d robotToCamArducamRight = new Transform3d(new Translation3d(0.2368, -0.2707, 0.2223),
      new Rotation3d(0, Units.degreesToRadians(-18.234),
          Units.degreesToRadians(7.2367)));
}
