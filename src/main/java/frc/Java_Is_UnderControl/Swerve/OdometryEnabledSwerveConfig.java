package frc.Java_Is_UnderControl.Swerve;

import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;

public class OdometryEnabledSwerveConfig extends BaseSwerveConfig {

  public final PoseEstimator autonomousPoseEstimator;

  public final PoseEstimator teleoperatedPoseEstimator;

  public final MoveToPosePIDConfig moveToPosePIDConfig;

  public OdometryEnabledSwerveConfig(double maxRotationSpeed,
      SwervePathPlannerConfig pathPlannerConfig, PoseEstimator autonomousPoseEstimator,
      PoseEstimator teleoperatedPoseEstimator, PIDConfig headingPIDConfig, MoveToPosePIDConfig moveToPosePIDConfig) {
    super(maxRotationSpeed, pathPlannerConfig, headingPIDConfig);
    this.autonomousPoseEstimator = autonomousPoseEstimator;
    this.teleoperatedPoseEstimator = teleoperatedPoseEstimator;
    this.moveToPosePIDConfig = moveToPosePIDConfig;
  }

}
