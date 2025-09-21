package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;

public class ComparatorPose2D implements Comparator<Pose2d> {
  @Override
  public int compare(Pose2d o1, Pose2d o2) {
    double sum1 = o1.getX() + o1.getY();
    double sum2 = o2.getX() + o2.getY();
    int sum = (int) (Math.round(sum1) - Math.round(sum2));
    return sum;
  }

}
