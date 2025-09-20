package frc.Java_Is_UnderControl.Swerve;

import frc.Java_Is_UnderControl.Control.PIDConfig;

public class BaseSwerveConfig {

  public final double maxRotationRate;// Use fractions, example: 0.75 = 3/4 of a rotation

  public final SwervePathPlannerConfig pathPlannerConfig;

  public final PIDConfig headingPidConfig;

  public BaseSwerveConfig(double maxRotationSpeed,
      SwervePathPlannerConfig pathPlannerConfig, PIDConfig pidHeading) {
    this.headingPidConfig = pidHeading;
    this.maxRotationRate = maxRotationSpeed;
    this.pathPlannerConfig = pathPlannerConfig;
  }
}
