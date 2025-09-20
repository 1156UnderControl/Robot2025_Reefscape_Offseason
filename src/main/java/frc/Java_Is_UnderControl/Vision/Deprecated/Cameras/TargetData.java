package frc.Java_Is_UnderControl.Vision.Deprecated.Cameras;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetData {
  private Pose2d targetPose;
  private double targetYaw;
  private double targetPitch;
  private double targetSkew;
  private double targetArea;
  private double distanceToTarget;

  public TargetData(
      double targetYaw,
      double targetPitch,
      double targetSkew,
      double targetArea,
      double distanceToTarget) {

    this.targetYaw = targetYaw;
    this.targetPitch = targetPitch;
    this.targetSkew = targetSkew;
    this.targetArea = targetArea;
    this.distanceToTarget = distanceToTarget;
  }

  public double getYaw() {
    return this.targetYaw;
  }

  public double getPitch() {
    return this.targetPitch;
  }

  public double getSkew() {
    return this.targetSkew;
  }

  public double getArea() {
    return this.targetArea;
  }

  public double getDistanceTarget() {
    return this.distanceToTarget;
  }
}
