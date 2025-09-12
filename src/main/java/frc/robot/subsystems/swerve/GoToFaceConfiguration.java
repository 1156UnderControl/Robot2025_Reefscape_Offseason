package frc.robot.subsystems.swerve;

public class GoToFaceConfiguration {

  public double minErrorPose;
  public double maxErrorPose;
  public double maxVelocity;
  public double minVelocity;
  public String goToBranchMode;

  public GoToFaceConfiguration(double minErrorPose,
      double maxErrorPose, String goToBranchMode,
      double minVelocity, double maxVelocity) {
    this.minErrorPose = minErrorPose;
    this.maxErrorPose = maxErrorPose;
    this.goToBranchMode = goToBranchMode;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }
}
