package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Comparator;

public class ComparatorPoseEstimation implements Comparator<PoseEstimation> {
  @Override
  public int compare(PoseEstimation o1, PoseEstimation o2) {
    double sum1 = o1.estimatedPose.getX() + o1.estimatedPose.getY();
    double sum2 = o2.estimatedPose.getX() + o2.estimatedPose.getY();
    int sum = (int) (Math.round(sum1) - Math.round(sum2));
    return sum;
  }

}
