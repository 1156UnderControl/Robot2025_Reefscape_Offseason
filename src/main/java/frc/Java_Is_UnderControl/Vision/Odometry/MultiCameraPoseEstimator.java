package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;

public class MultiCameraPoseEstimator implements PoseEstimator {

  List<PoseEstimator> poseEstimatorsList;

  int numberOfTargetsUsed = 0;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomDoubleLogger stdDevXYLogger;

  CustomDoubleLogger stdDevThetaLogger;

  String multiPoseEstimatorName;

  public MultiCameraPoseEstimator(List<PoseEstimator> poseEstimators, String multiPoseEstimatorName) {
    this.poseEstimatorsList = poseEstimators;
    this.multiPoseEstimatorName = multiPoseEstimatorName;
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/MultiCameraEstimator/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger("/Vision/MultiCameraEstimator/NumberOfDetectedTags");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    ArrayList<Pose2d> pose2Ds = new ArrayList<>();
    ArrayList<PoseEstimation> poseEstimations = new ArrayList<>();
    for (PoseEstimator poseEstimator : this.poseEstimatorsList) {
      Optional<PoseEstimation> possiblePoseEstimation = poseEstimator.getEstimatedPose(referencePose);
      if (possiblePoseEstimation.isPresent()) {
        PoseEstimation poseEstimation = possiblePoseEstimation.get();
        for (int i = 0; i < poseEstimation.numberOfTargetsUsed; i++) {
          pose2Ds.add(poseEstimation.estimatedPose.toPose2d());
          poseEstimations.add(poseEstimation);
        }
      }
    }
    if (poseEstimations.size() < 1) {
      return Optional.empty();
    }
    PoseEstimation poseEstimation;
    poseEstimations.sort(new ComparatorPoseEstimationTagCount());
    if (poseEstimations.size() > 1) {
      double TagCount1 = poseEstimations.get(poseEstimations.size() - 1).numberOfTargetsUsed;
      double TagCount2 = poseEstimations.get(poseEstimations.size() - 2).numberOfTargetsUsed;
      if (TagCount1 != TagCount2) {
        double biggerDistance = Math.max(poseEstimations.get(poseEstimations.size() - 1).distanceToTag,
            poseEstimations.get(poseEstimations.size() - 2).distanceToTag);
        if (biggerDistance == poseEstimations.get(poseEstimations.size() - 1).distanceToTag) {
          poseEstimation = poseEstimations.get(poseEstimations.size() - 2);
        } else {
          poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
        }
      } else {
        poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
      }
    } else {
      poseEstimation = poseEstimations.get(poseEstimations.size() - 1);
    }
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    return Optional.of(poseEstimation);
  }

  @Override
  public String getEstimatorName() {
    return this.multiPoseEstimatorName;
  }
}
