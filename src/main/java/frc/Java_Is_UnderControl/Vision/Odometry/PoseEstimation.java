package frc.Java_Is_UnderControl.Vision.Odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** An estimated pose based on pipeline result */
public class PoseEstimation {
  /** The estimated pose */
  public final Pose3d estimatedPose;

  /** The estimated time the frame used to derive the robot pose was taken */
  public final double timestampSeconds;

  /** A list of the targets used to compute this pose */
  public int numberOfTargetsUsed;

  /** The distance from the tag */
  public final double distanceToTag;

  public Matrix<N3, N1> visionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);

  /**
   * Constructs an EstimatedRobotPose
   *
   * @param estimatedPose    estimated pose
   * @param timestampSeconds timestamp of the estimate
   */
  public PoseEstimation(
      Pose3d estimatedPose,
      double timestampSeconds,
      int numberOfTargetsUsed, double distanceToTag, Matrix<N3, N1> visionStdDev) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
    this.numberOfTargetsUsed = numberOfTargetsUsed;
    this.distanceToTag = distanceToTag;
    this.visionStdDev = visionStdDev;
  }

  public PoseEstimation(
      Pose3d estimatedPose,
      double timestampSeconds,
      int numberOfTargetsUsed, double distanceToTag) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
    this.numberOfTargetsUsed = numberOfTargetsUsed;
    this.distanceToTag = distanceToTag;
  }

  public void changeVisionStdDev(Matrix<N3, N1> visionStdDev) {
    this.visionStdDev = visionStdDev;
  }
}
