package frc.Java_Is_UnderControl.Swerve.Assist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;

public class DriveAssist {
  CustomDoubleLogger angleDifferenceLogger = new CustomDoubleLogger(
      "DriveAssist/PerfectToRequestedAngleDifference");
  CustomDoubleLogger perfectTranslationAngleLogger = new CustomDoubleLogger(
      "DriveAssist/PerfectTranslationAngle");
  CustomDoubleLogger requestedTranslationAngleLogger = new CustomDoubleLogger(
      "DriveAssist/RequestedTranslationAngle");
  CustomPose2dLogger robotPoseLogger = new CustomPose2dLogger("DriveAssist/RobotPose");
  CustomPose2dLogger objectPoseLogger = new CustomPose2dLogger("DriveAssist/ObjectPose");
  CustomStringLogger stateLogger = new CustomStringLogger("DriveAssist/State");

  Translation2d correctionReferencePosition;
  double correntionPercentage;
  double maxDistanceToCorrect;
  double maxJoystickAngleToCorrect;

  public DriveAssist() {
    this(new Translation2d(), 1, 2, 45);
  }

  public DriveAssist(Translation2d correctionReferencePosition, double correctionPercentage,
      double maxDistanceToCorrect, double maxJoystickAngleToCorrect) {
    this.correctionReferencePosition = correctionReferencePosition;
    this.correntionPercentage = correctionPercentage;
    this.maxDistanceToCorrect = maxDistanceToCorrect;
    this.maxJoystickAngleToCorrect = maxJoystickAngleToCorrect;
  }

  public ChassisSpeeds getCorrectedDesiredSpeedToTranslation(Pose2d robotPose, Pose2d objectPose,
      ChassisSpeeds robotVelocity) {
    robotPoseLogger.appendRadians(robotPose);
    objectPoseLogger.appendRadians(objectPose);
    if (robotPose.getTranslation().getDistance(objectPose.getTranslation()) > this.maxDistanceToCorrect) {
      stateLogger.append("REJECTED_TOO_FAR_FROM_OBJECT");
      return robotVelocity;
    }
    Translation2d correctedReferencePose = CoordinatesTransform
        .fromRobotRelativeToF(robotPose, new Pose2d(correctionReferencePosition, new Rotation2d()))
        .getTranslation();
    Translation2d perfectTranslationDirection = objectPose.getTranslation().minus(correctedReferencePose);
    Rotation2d perfectTranslationAngle = perfectTranslationDirection.getAngle();
    this.perfectTranslationAngleLogger.append(perfectTranslationAngle.getDegrees());
    Rotation2d requestedTranslationAngle = new Translation2d(robotVelocity.vx,
        robotVelocity.vy).getAngle();
    this.requestedTranslationAngleLogger.append(requestedTranslationAngle.getDegrees());
    Rotation2d requestedToPerfectAngleDifference = perfectTranslationAngle.minus(requestedTranslationAngle);
    angleDifferenceLogger.append(requestedToPerfectAngleDifference.getDegrees());
    if (Math.abs(requestedToPerfectAngleDifference.getDegrees()) > this.maxJoystickAngleToCorrect) {
      stateLogger.append("REJECTED_ANGLE_REQUESTED_TOO_FAR_FROM_PERFECT");
      return robotVelocity;
    }
    Rotation2d desiredTranslationAngle = requestedTranslationAngle
        .plus(requestedToPerfectAngleDifference.times(correntionPercentage));
    double robotVelocityIntensity = Math.hypot(robotVelocity.vx,
        robotVelocity.vy);
    double desiredVx = robotVelocityIntensity * desiredTranslationAngle.getCos();
    double desiredVy = robotVelocityIntensity * desiredTranslationAngle.getSin();
    stateLogger.append("RETURNING_CORRECTED_SPEEDS");
    return new ChassisSpeeds(desiredVx, desiredVy,
        robotVelocity.omega);
  }

}
