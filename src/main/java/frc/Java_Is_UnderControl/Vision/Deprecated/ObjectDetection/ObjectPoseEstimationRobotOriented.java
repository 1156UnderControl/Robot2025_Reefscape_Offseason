package frc.Java_Is_UnderControl.Vision.Deprecated.ObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose3dLogger;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;

public class ObjectPoseEstimationRobotOriented {
  ObjectDetection object;

  ObjectDetectionCamera camera;

  CustomDoubleLogger cameraAngleLogger;

  CustomDoubleLogger tyLogger;

  CustomDoubleLogger txLogger;

  CustomDoubleLogger txDegreesLogger;

  CustomDoubleLogger tyDegreesLogger;

  CustomDoubleLogger distanceToObjectLogger;

  CustomDoubleLogger xObjectSDistanceLogger;

  CustomDoubleLogger yObjectDistanceLogger;

  CustomPose3dLogger poseOBjectInTheFieldLogger;

  double angleObjectInMaxTy;

  double angleObjectInMaxTx;

  public ObjectPoseEstimationRobotOriented(ObjectDetection object, ObjectDetectionCamera camera,
      double angleObjectInMaxTy, double angleObjectInMaxTx) {
    this.object = object;
    this.camera = camera;
    this.angleObjectInMaxTy = angleObjectInMaxTy;
    this.angleObjectInMaxTx = angleObjectInMaxTx;
    setupLogs();
  }

  private void setupLogs() {
    cameraAngleLogger = new CustomDoubleLogger("ObjectDetection/Camera Angle");
    tyLogger = new CustomDoubleLogger("ObjectDetection/Raw tY Value");
    txLogger = new CustomDoubleLogger("ObjectDetection/Raw tX Value");
    tyDegreesLogger = new CustomDoubleLogger("ObjectDetection/tY In Degrees");
    txDegreesLogger = new CustomDoubleLogger("ObjectDetection/tX In Degrees");
    distanceToObjectLogger = new CustomDoubleLogger("ObjectDetection/Camera Distance To Object");
    xObjectSDistanceLogger = new CustomDoubleLogger("ObjectDetection/X Object Distance To Robot");
    yObjectDistanceLogger = new CustomDoubleLogger("ObjectDetection/Y Object Distance To Robot");
    poseOBjectInTheFieldLogger = new CustomPose3dLogger("ObjectDetection/Note Pose In The Field");
  }

  public Pose2d getObjectPoseInField(Pose2d robotPose, Pose2d objectPose) {
    Pose2d poseInTheField2d = CoordinatesTransform.fromRobotRelativeToF(robotPose, objectPose);
    Pose3d poseInTheField3d = new Pose3d(poseInTheField2d);
    poseOBjectInTheFieldLogger.appendRadians(poseInTheField3d);
    return poseInTheField2d;
  }

  public Pose2d getObjectPoseFromRobotCenter() {
    return new Pose2d(getXnoteDistanceFromRobotCenter(), -getYnoteDistanceFromRobotCenter(), new Rotation2d());
  }

  public double getXnoteDistanceFromRobotCenter() {
    double calculation = (getDistanceFromObject() * Math.cos(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getX();
    xObjectSDistanceLogger.append(calculation);
    return calculation;
  }

  public double getYnoteDistanceFromRobotCenter() {
    double calculation = (getDistanceFromObject() * Math.sin(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getY();
    yObjectDistanceLogger.append(calculation);
    return calculation;
  }

  public double getDistanceFromObject() {
    cameraAngleLogger.append(camera.getCameraAngle());
    double calculation = camera.cameraHeight
        * Math.tan(Units.degreesToRadians(getObjectAngleY()) + Units.degreesToRadians(camera.getCameraAngle()));
    distanceToObjectLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleY() {
    tyLogger.append(object.ty);
    double calculation = (object.ty * angleObjectInMaxTy) / camera.maxTy;
    tyDegreesLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleX() {
    txLogger.append(object.tx);
    double calculation = (object.tx * angleObjectInMaxTx) / camera.maxTx;
    txDegreesLogger.append(calculation);
    return calculation;
  }

}
