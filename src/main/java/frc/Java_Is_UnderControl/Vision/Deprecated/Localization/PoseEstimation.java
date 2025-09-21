package frc.Java_Is_UnderControl.Vision.Deprecated.Localization;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.TargetData;

public class PoseEstimation {

  AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(null);
  int numberOfTags = layout.getTags().size();
  TargetData data;
  int dataID;

  public PoseEstimation(Map<Integer, TargetData> mapTargetData) {
    for (int id = 0; id <= numberOfTags; id++) {
      var dataFromApril = mapTargetData.get(id);
      if (dataFromApril != null) {
        this.data = dataFromApril;
        this.dataID = id;
      }
    }
  }

  private void robotPoseCalc() {

  }

  public double getTargetPitch() {
    return this.data.getPitch();
  }

  public double getYaw() {
    return this.data.getYaw();
  }

  public double getSkew() {
    return this.data.getSkew();
  }

  public double getDistanceTarget() {
    return this.data.getDistanceTarget();
  }

  public int getAprilID() {
    return this.dataID;
  }
}
