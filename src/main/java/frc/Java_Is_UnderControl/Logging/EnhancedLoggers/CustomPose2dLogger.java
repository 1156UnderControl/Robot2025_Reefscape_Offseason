package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.datalog.StructLogEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class CustomPose2dLogger {

  private static boolean isFmsMatch;

  // private String name;

  private Pose2d loggedValue;

  private StructPublisher<Pose2d> publisher;

  private StructLogEntry<Pose2d> logEntry;

  public CustomPose2dLogger(String name) {
    // super(DataLogManager.getLog(), name);
    // this.name = name;
    this.logEntry = StructLogEntry.create(DataLogManager.getLog(), name, Pose2d.struct);
    this.publisher = NetworkTableInstance.getDefault()
        .getStructTopic(name, Pose2d.struct).publish();
    CustomPose2dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = new Pose2d(new Translation2d(100, 100), new Rotation2d()); // Set to something different than
    this.appendRadians(new Pose2d());
  }

  public void appendRadians(Pose2d pose) {
    if (!pose.equals(this.loggedValue)) {
      this.loggedValue = pose;
      this.logEntry.append(pose);
      if (!CustomPose2dLogger.isFmsMatch) {
        publisher.set(pose);
      }
    }
  }

  public void appendDegrees(Pose2d pose) {
    this.logEntry.append(pose);
    if (!CustomPose2dLogger.isFmsMatch) {
      publisher.set(pose);
    }
  }
}
