package frc.Java_Is_UnderControl.Swerve.Configs;

import frc.Java_Is_UnderControl.Control.PIDConfig;

public class BaseSwerveConfig {

  public final double maxRotationRateInFractions;

  public final SwervePathPlannerConfig pathPlannerConfig;

  public final PIDConfig headingPidConfig;

  public BaseSwerveConfig(
      double maxRotationSpeedInFractions,
      SwervePathPlannerConfig pathPlannerConfig,
      PIDConfig pidHeading) {
    this.headingPidConfig = pidHeading;
    this.maxRotationRateInFractions = maxRotationSpeedInFractions;
    this.pathPlannerConfig = pathPlannerConfig;
  }
}
