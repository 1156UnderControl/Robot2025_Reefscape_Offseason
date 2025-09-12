package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Comparator;

public class ComparatorPoseEstimationTagCount implements Comparator<PoseEstimation> {
  @Override
  public int compare(PoseEstimation o1, PoseEstimation o2) {
    return o1.numberOfTargetsUsed - o2.numberOfTargetsUsed;
  }

}
