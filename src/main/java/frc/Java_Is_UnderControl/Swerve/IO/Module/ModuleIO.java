package frc.Java_Is_UnderControl.Swerve.IO.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public boolean steerEncoderConnected = false;
    public Rotation2d steerAbsolutePosition = new Rotation2d();
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void setDriveOpenLoop(double output);

  public void setSteerOpenLoop(double output);

  public void setDriveVelocity(double velocityMetersperSecond);

  public void setSteerPosition(Rotation2d rotation);

  public SwerveModuleState getCurrentModuleState();
}
