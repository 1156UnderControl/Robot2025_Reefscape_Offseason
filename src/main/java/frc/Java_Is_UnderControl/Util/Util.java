package frc.Java_Is_UnderControl.Util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;

public class Util {

  private Util() {
  }

  public static boolean withinHypotDeadband(double x, double y) {
    return Math.hypot(x, y) < 0.5;
  }

  public static Boolean inRange(double x, double minX, double maxX) {
    return x > minX && x < maxX;
  }

  public static Boolean atSetpoint(double processVariable, double setpoint, double deadband) {
    return (processVariable > setpoint - deadband) && (processVariable < setpoint + deadband);
  }

  public static Pose2d medianPose2Dlist(List<Pose2d> list) {
    Pose2d pose;
    if (list.size() % 2 == 0) {
      pose = averageTwoPose2Ds(list.get(list.size() / 2), list.get(list.size() / 2 - 1));
    } else {
      pose = list.get(list.size() / 2);
    }
    return pose;
  }

  public static PoseEstimation medianPoseEstimationsList(List<PoseEstimation> list) {
    PoseEstimation pose;
    if (list.size() % 2 == 0) {
      pose = averageTwoPoseEstimations(list.get(list.size() / 2), list.get(list.size() / 2 - 1));
    } else {
      pose = list.get(list.size() / 2);
    }
    return pose;
  }

  private static PoseEstimation averageTwoPoseEstimations(PoseEstimation pose1, PoseEstimation pose2) {
    double x = (pose1.estimatedPose.getX() + pose2.estimatedPose.getX()) / 2;
    double y = (pose1.estimatedPose.getY() + pose2.estimatedPose.getY()) / 2;
    double rot = (pose1.estimatedPose.getRotation().getAngle() + pose2.estimatedPose.getRotation().getAngle()) / 2;
    int numberOfTargets = (pose1.numberOfTargetsUsed + pose2.numberOfTargetsUsed) / 2;
    double timestamp = (pose1.timestampSeconds + pose1.timestampSeconds) / 2;
    double distanceToTag = (pose1.distanceToTag + pose1.distanceToTag) / 2;
    return new PoseEstimation(new Pose3d(new Pose2d(x, y, new Rotation2d(rot))), timestamp, numberOfTargets,
        distanceToTag);
  }

  private static Pose2d averageTwoPose2Ds(Pose2d pose1, Pose2d pose2) {
    double x = (pose1.getX() + pose2.getX()) / 2;
    double y = (pose1.getY() + pose2.getY()) / 2;
    double rot = (pose1.getRotation().getRadians() + pose2.getRotation().getRadians()) / 2;
    return new Pose2d(x, y, new Rotation2d(rot));
  }
}
