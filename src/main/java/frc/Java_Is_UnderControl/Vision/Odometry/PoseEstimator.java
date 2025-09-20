package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public interface PoseEstimator {

  public String getEstimatorName();

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose);

}
