package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Java_Is_UnderControl.Logging.Pose3dLogEntry;

public class CustomPose3dLogger extends Pose3dLogEntry {

  private static boolean isFmsMatch;

  private String name;

  private Pose3d loggedValue;

  private StructPublisher<Pose3d> publisher;

  public CustomPose3dLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    this.publisher = NetworkTableInstance.getDefault()
        .getStructTopic(name, Pose3d.struct).publish();
    CustomPose3dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = new Pose3d(100, 100, 0, new Rotation3d()); // Set to something different than default for initial
    // // logging
    this.publisher = NetworkTableInstance.getDefault()
        .getStructTopic(name, Pose3d.struct).publish();
    this.appendRadians(new Pose3d());
  }

  @Override
  public void appendRadians(Pose3d pose) {
    if (DriverStation.isEnabled() && !pose.equals(this.loggedValue)) {
      this.loggedValue = pose;
      super.appendRadians(pose);
      if (!CustomPose3dLogger.isFmsMatch) {
        publisher.set(pose);
      }
    }
  }

  @Override
  public void appendDegrees(Pose3d pose) {
    super.appendDegrees(pose);
    if (CustomPose3dLogger.isFmsMatch) {
      publisher.set(pose);
    }
  }
}
