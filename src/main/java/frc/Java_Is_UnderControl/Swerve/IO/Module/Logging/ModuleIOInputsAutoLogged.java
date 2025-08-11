package frc.Java_Is_UnderControl.Swerve.IO.Module.Logging;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.Java_Is_UnderControl.Swerve.IO.Module.ModuleIO;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("/DriveConnected", driveConnected);
    table.put("/DrivePositionRad", drivePositionRad);
    table.put("/DriveVelocityRadPerSec", driveVelocityRadPerSec);
    table.put("/DriveAppliedVolts", driveAppliedVolts);
    table.put("/DriveCurrentAmps", driveCurrentAmps);
    table.put("/SteerConnected", steerConnected);
    table.put("/SteerEncoderConnected", steerEncoderConnected);
    table.put("/SteerAbsolutePosition", steerAbsolutePosition);
    table.put("/SteerPosition", steerPosition);
    table.put("/SteerVelocityRadPerSec", steerVelocityRadPerSec);
    table.put("/SteerAppliedVolts", steerAppliedVolts);
    table.put("/SteerCurrentAmps", steerCurrentAmps);
    table.put("/OdometryTimestamps", odometryTimestamps);
    table.put("/OdometryDrivePositionsRad", odometryDrivePositionsRad);
    table.put("/OdometrySteerPositions", odometrySteerPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    driveConnected = table.get("/DriveConnected", driveConnected);
    drivePositionRad = table.get("/DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.get("/DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveAppliedVolts = table.get("/DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.get("/DriveCurrentAmps", driveCurrentAmps);
    steerConnected = table.get("/SteerConnected", steerConnected);
    steerEncoderConnected = table.get("/SteerEncoderConnected", steerEncoderConnected);
    steerAbsolutePosition = table.get("/SteerAbsolutePosition", steerAbsolutePosition);
    steerPosition = table.get("/SteerPosition", steerPosition);
    steerVelocityRadPerSec = table.get("/SteerVelocityRadPerSec", steerVelocityRadPerSec);
    steerAppliedVolts = table.get("/SteerAppliedVolts", steerAppliedVolts);
    steerCurrentAmps = table.get("/SteerCurrentAmps", steerCurrentAmps);
    odometryTimestamps = table.get("/OdometryTimestamps", odometryTimestamps);
    odometryDrivePositionsRad = table.get("/OdometryDrivePositionsRad", odometryDrivePositionsRad);
    odometrySteerPositions = table.get("/OdometrySteerPositions", odometrySteerPositions);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.moduleName = this.moduleName;
    copy.driveConnected = this.driveConnected;
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveCurrentAmps = this.driveCurrentAmps;
    copy.steerConnected = this.steerConnected;
    copy.steerEncoderConnected = this.steerEncoderConnected;
    copy.steerAbsolutePosition = this.steerAbsolutePosition;
    copy.steerPosition = this.steerPosition;
    copy.steerVelocityRadPerSec = this.steerVelocityRadPerSec;
    copy.steerAppliedVolts = this.steerAppliedVolts;
    copy.steerCurrentAmps = this.steerCurrentAmps;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositionsRad = this.odometryDrivePositionsRad.clone();
    copy.odometrySteerPositions = this.odometrySteerPositions.clone();
    return copy;
  }
}
