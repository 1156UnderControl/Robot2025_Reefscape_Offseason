package frc.Java_Is_UnderControl.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;

public class CoordinatesTransform {
  /**
   * Moves a given Pose3d backward by a specified distance.
   *
   * @param pose The original Pose2d.
   * @param distance The distance to move backward.
   * @return A new Pose2d moved backward.
   */
  public static Pose2d getRetreatPose(Pose2d pose, double distance) {
    Transform2d backwardsTransform = new Transform2d(distance, 0, new Rotation2d());
    return pose.plus(backwardsTransform);
  }

  /**
   * Moves a given Pose3d backward by a specified distance.
   *
   * @param pose The original Pose2d.
   * @param distance The distance to move forward.
   * @return A new Pose2d moved forward.
   */
  public static Pose2d getForwardPose(Pose2d pose, double distance) {
    Transform2d forwardTransform = new Transform2d(distance, 0, new Rotation2d());
    return pose.plus(forwardTransform);
  }

  public static Pose3d applyRotationToPoseAngle(Pose3d pose, Rotation3d rotation) {
    return new Pose3d(pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().rotateBy(rotation));
  }

  public static Pose2d applyRotationToPoseAngle(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().rotateBy(rotation));
  }

  public static Pose2d fromRobotRelativeToF(Pose2d robotPose, Pose2d poseForConversion) {
    Pose2d poseInTheField =
        robotPose.plus(
            new Transform2d(poseForConversion.getTranslation(), poseForConversion.getRotation()));
    return poseInTheField;
  }
}
