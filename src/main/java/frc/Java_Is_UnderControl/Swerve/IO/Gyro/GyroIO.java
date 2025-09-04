package frc.Java_Is_UnderControl.Swerve.IO.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPosition = 0;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public void updateInputs(GyroIOInputs inputs);

  public GyroIOInputs getInputs();

  public void setYaw(double yaw);

  public double getYaw();
}
