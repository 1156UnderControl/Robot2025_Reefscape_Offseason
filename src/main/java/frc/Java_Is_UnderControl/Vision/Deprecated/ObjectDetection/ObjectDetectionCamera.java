package frc.Java_Is_UnderControl.Vision.Deprecated.ObjectDetection;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionCamera {
  public final double cameraHeight;

  public final double maxTy;

  public final double maxTx;

  // wpi cordinates
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system
  public Transform3d robotToCamera;

  public ObjectDetectionCamera(Transform3d robotToCamera, double maxTy, double maxTx) {
    this.robotToCamera = robotToCamera;
    this.cameraHeight = robotToCamera.getZ();
    this.maxTx = maxTx;
    this.maxTy = maxTy;
  }

  // get camera angle as the 0 pointing towards the ground
  public double getCameraAngle() {
    return 90 - Units.radiansToDegrees(robotToCamera.getRotation().getY());
  }
}
