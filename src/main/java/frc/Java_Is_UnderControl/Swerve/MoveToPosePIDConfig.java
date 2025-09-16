package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.Java_Is_UnderControl.Control.PIDConfig;

public class MoveToPosePIDConfig {

  public PIDConfig pidTranslation;

  public Constraints constraintsTranslation;

  public PIDConfig pidX;

  public Constraints constraintsX;

  public PIDConfig pidY;

  public Constraints constraintsY;

  public MoveToPosePIDConfig(PIDConfig pidX, Constraints constraintsX, PIDConfig pidY, Constraints constraintsY) {
    this.pidX = pidX;
    this.constraintsX = constraintsX;
    this.pidY = pidY;
    this.constraintsY = constraintsY;
  }

  public MoveToPosePIDConfig(PIDConfig pidTranslation, Constraints constraints) {
    this(pidTranslation, constraints, pidTranslation, constraints);
    this.pidTranslation = pidTranslation;
    this.constraintsTranslation = constraints;
  }

  public PIDController getPidTranslation() {
    PIDController pidTranslationController = new PIDController(pidTranslation.kP, pidTranslation.kI, pidTranslation.kD);
    pidTranslationController.setIZone(this.pidTranslation.iZone);
    return pidTranslationController;
  }

  public PIDController getPidX() {
    return new PIDController(pidX.kP, pidX.kI, pidX.kD);
  }

  public PIDController getPidY() {
    return new PIDController(pidY.kP, pidY.kI, pidY.kD);
  }

  public ProfiledPIDController getProfiledPIDX() {
    return new ProfiledPIDController(pidX.kP, pidX.kI, pidX.kD, constraintsX);
  }

  public ProfiledPIDController getProfiledPIDY() {
    return new ProfiledPIDController(pidY.kP, pidY.kI, pidY.kD, constraintsY);
  }

}
