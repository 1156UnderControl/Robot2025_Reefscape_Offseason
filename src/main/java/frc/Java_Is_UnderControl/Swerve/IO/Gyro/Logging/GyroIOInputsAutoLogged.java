package frc.Java_Is_UnderControl.Swerve.IO.Gyro.Logging;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.Java_Is_UnderControl.Swerve.IO.Gyro.GyroIO;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("/Pigeon/isConnected", connected);
    table.put("/Pigeon/YawPosition", yawPosition);
    table.put("/Pigeon/YawVelocityRadPerSec", yawVelocityRadPerSec);
    table.put("/Pigeon/OdometryYawTimestamps", odometryYawTimestamps);
    table.put("/Pigeon/OdometryYawPositions", odometryYawPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("/Pigeon/isConnected", connected);
    yawPosition = table.get("/Pigeon/YawPosition", yawPosition);
    yawVelocityRadPerSec = table.get("/Pigeon/YawVelocityRadPerSec", yawVelocityRadPerSec);
    odometryYawTimestamps = table.get("/Pigeon/OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("/Pigeon/OdometryYawPositions", odometryYawPositions);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yawPosition = this.yawPosition;
    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    return copy;
  }
}
