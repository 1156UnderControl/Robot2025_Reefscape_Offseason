package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers.PoseEstimate;
import frc.robot.constants.VisionConstants;

public class LimelightPoseEstimator implements PoseEstimator {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  double fieldLength = Units.inchesToMeters(690.876);

  double fieldWidth = Units.inchesToMeters(317);

  double limitAngVelForUpdating;

  String limelightName;

  boolean only2TagsMeasurements = false;

  boolean useMegaTag1;

  private Boolean useVisionHeadingCorrection = false;

  CustomStringLogger stateOfPoseUpdate;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomDoubleLogger distToTag;

  CustomDoubleLogger headingMegaTag2;

  CustomDoubleLogger stdDevXYLogger;

  CustomDoubleLogger stdDevThetaLogger;

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements, boolean useMegaTag1,
      double limitAngVelForUpdating) {
    this.limelightName = limelightName;
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.useMegaTag1 = useMegaTag1;
    this.limitAngVelForUpdating = limitAngVelForUpdating;
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/IsDetectingTags");
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/NumberOfDetectedTags");
    this.headingMegaTag2 = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/Heading MegaTag2");
    this.distToTag = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DistanceToTag");
    this.stateOfPoseUpdate = new CustomStringLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/StateOfPoseUpdate");
    this.stdDevXYLogger = new CustomDoubleLogger("Vision/PhotonVisionPoseEstimator/" + limelightName + "/stdDevXY");
    this.stdDevThetaLogger = new CustomDoubleLogger(
        "Vision/PhotonVisionPoseEstimator/" + limelightName + "/stdDevTheta");
  }

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements) {
    this(limelightName, only2TagsMeasurements, false, 3);
  }

  public LimelightPoseEstimator(String limelightName) {
    this(limelightName, false);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      if (!LimelightHelpers.getTV(this.limelightName)
          || Math.abs(OdometryEnabledSwerveSubsystem.robotAngularVelocity) >= limitAngVelForUpdating) {
        this.isDetectingLogger.append(false);
        this.numberOfDetectedTagsLogger.append(0);
        this.stateOfPoseUpdate.append("WITHOUT_TARGET_OR_HIGH_ANGULAR_VELOCITY");
        return Optional.empty();
      }
      PoseEstimate limelightPoseEstimate;
      if (useMegaTag1) {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_1");
      } else {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_2");
      }
      if (limelightPoseEstimate == null) {
        return Optional.empty();
      }

      if (limelightPoseEstimate.tagCount == 1) {
        useVisionHeadingCorrection = false;
      } else {
        useVisionHeadingCorrection = false;
      }

      PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);
      this.isDetectingLogger.append(true);
      this.detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
      this.numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
      this.distToTag.append(poseEstimation.distanceToTag);
      if (poseOutOfField(poseEstimation)) {
        this.stateOfPoseUpdate.append("REJECTED_BY_OUT_OF_THE_FIELD_ESTIMATION");
        return Optional.empty();
      }
      if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
        this.stateOfPoseUpdate.append("REJECTED_BY_ONLY_2_TAGS_MEASUREMENTS");
        return Optional.empty();
      }

      return Optional.of(poseEstimation);
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private boolean poseOutOfField(PoseEstimation pose2D) {
    double x = pose2D.estimatedPose.getX();
    double y = pose2D.estimatedPose.getY();
    return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    double stdDevXY = VisionConstants.xyStdDevCoefficient * Math.pow(limelightPoseEstimate.avgTagDist, 2)
        / limelightPoseEstimate.tagCount;
    double stdDevTheta = useVisionHeadingCorrection
        ? VisionConstants.xyStdDevCoefficient * Math.pow(limelightPoseEstimate.avgTagDist, 2)
            / limelightPoseEstimate.tagCount
        : Double.POSITIVE_INFINITY;
    return new PoseEstimation(new Pose3d(limelightPoseEstimate.pose), limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount, limelightPoseEstimate.avgTagDist, VecBuilder.fill(stdDevXY,
            stdDevXY, stdDevTheta));
  }

  public String getEstimatorName() {
    return this.limelightName;
  }
}
