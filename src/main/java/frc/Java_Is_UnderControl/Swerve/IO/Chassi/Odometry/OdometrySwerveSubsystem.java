package frc.Java_Is_UnderControl.Swerve.IO.Chassi.Odometry;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Swerve.Configs.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.IO.Chassi.Base.BaseSwerveSubsystem;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;

public abstract class OdometrySwerveSubsystem extends BaseSwerveSubsystem {

  public static double robotOrientation;

  public static double robotAngularVelocity;

  private PoseEstimator autonomousPoseEstimator;

  private PoseEstimator teleopPoseEstimator;

  private PoseEstimator defaultAutonomousPoseEstimator;

  private PoseEstimator defaultTeleopPoseEstimator;

  private PIDController moveToPoseXAxisPid;

  private PIDController moveToPoseYAxisPid;

  private Pose2d targetPose;

  private Pose2d targetAimPose;

  // Create the constraints to use while pathfinding. The constraints defined in
  // the path will only be used for the path.
  PathConstraints constraints;

  public OdometrySwerveSubsystem(OdometryEnabledSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(config, drivetrainConstants, modules);
    this.moveToPoseXAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.moveToPoseYAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.constraints = config.pathPlannerConfig.pathFinderConstraints;
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.defaultAutonomousPoseEstimator = config.teleoperatedPoseEstimator;
    this.defaultAutonomousPoseEstimator = config.autonomousPoseEstimator;
    this.defaultTeleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
  }

  public OdometrySwerveSubsystem(OdometryEnabledSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(config, drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
        modules);
    this.moveToPoseXAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.moveToPoseYAxisPid = config.moveToPosePIDConfig.getPidTranslation();
    this.constraints = config.pathPlannerConfig.pathFinderConstraints;
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.defaultAutonomousPoseEstimator = config.autonomousPoseEstimator;
    this.defaultTeleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
  }

  private void updateOdometry() {
    if (DriverStation.isAutonomousEnabled() || DriverStation.isAutonomous()) {
      this.updateOdometryWithPoseEstimator(this.autonomousPoseEstimator);
    } else {
      this.updateOdometryWithPoseEstimator(this.teleopPoseEstimator);
    }
  }

  protected void overrideTeleOpPoseEstimator(PoseEstimator poseEstimator) {
    if (poseEstimator == null) {
      this.teleopPoseEstimator = this.defaultTeleopPoseEstimator;
      return;
    }
    this.teleopPoseEstimator = poseEstimator;
  }

  protected void overrideAutonomousPoseEstimator(PoseEstimator poseEstimator) {
    if (poseEstimator == null) {
      this.autonomousPoseEstimator = this.defaultAutonomousPoseEstimator;
      return;
    }
    this.autonomousPoseEstimator = poseEstimator;
  }

  private void updateOdometryWithPoseEstimator(PoseEstimator poseEstimator) {
    Pose2d referencePose = getPose();
    Optional<PoseEstimation> possibleEstimatedPose = poseEstimator.getEstimatedPose(referencePose);
    if (possibleEstimatedPose.isPresent()) {
      PoseEstimation estimatedPose = possibleEstimatedPose.get();
      Pose2d poseVision = estimatedPose.estimatedPose.toPose2d();
      Logger.recordOutput("Subsystems/Swerve/Odometry/poseVision", poseVision);
      this.setVisionStdDev(estimatedPose.visionStdDev);
      this.addVisionMeasurement(poseVision, estimatedPose.timestampSeconds);
      Logger.recordOutput("Subsystems/Swerve/Odometry/poseEstimatorName", poseEstimator.getEstimatorName());
    }
  }

  protected double getDistanceToPosition(Translation2d targetPosition) {
    return this.getPose().getTranslation().getDistance(targetPosition);
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.driveAimingAtPosition(0, targetSpeeds, targetPosition, offsetAngle);
  }

  protected void driveAimingAtPosition(double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    Rotation2d targetAngle = this.getHeadingAimingAtPosition(dt, targetPosition, offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }

  protected void driveAimingAtPosition(double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(dt, targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected Rotation2d getHeadingAimingAtPosition(double dt, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getEarlyPoseMoving(dt).getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    return targetAngle;
  }

  protected void driveToPose(Pose2d targetPose) {
    this.driveToPose(targetPose, Double.POSITIVE_INFINITY);
  }

  protected void driveToPoseAimingAtPosition(double dt, Pose2d targetPose,
      Translation2d targetTranslationToAim, double maxSpeed) {
    this.driveToPoseAimingAtPosition(dt, targetPose, targetTranslationToAim, maxSpeed, 0.0);
  }

  protected void driveToPoseAimingAtPosition(double dt, Pose2d targetPose,
      Translation2d targetTranslationToAim, double maxSpeed, double offset) {
    Rotation2d targetAngle = this.getHeadingAimingAtPosition(dt, targetTranslationToAim,
        Rotation2d.fromDegrees(offset));
    targetAngle = AllianceFlipUtil.apply(targetAngle);
    Pose2d targetPoseWithLockedAngle = new Pose2d(targetPose.getX(), targetPose.getY(), targetAngle);
    this.driveToPose(targetPoseWithLockedAngle, maxSpeed);
  }

  protected void driveToPose(Pose2d targetPose, double maxSpeed) {
    Pose2d currentPose = this.getPose();
    double targetXVelocity = this.moveToPoseXAxisPid.calculate(currentPose.getX(),
        (targetPose.getX()));
    double targetYVelocity = this.moveToPoseYAxisPid.calculate(currentPose.getY(),
        (targetPose.getY()));
    double targetTranslationVelocity = new Translation2d(targetXVelocity, targetYVelocity).getNorm();
    if (targetTranslationVelocity > maxSpeed) {
      double conversionFactor = maxSpeed / targetTranslationVelocity;
      targetXVelocity = targetXVelocity * conversionFactor;
      targetYVelocity = targetYVelocity * conversionFactor;
    }
    Logger.recordOutput("Subsystems/Swerve/Odometry/targetXVelocity", targetXVelocity);
    Logger.recordOutput("Subsystems/Swerve/Odometry/targetYVelocity", targetYVelocity);
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(targetXVelocity, targetYVelocity, 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation());
    this.targetPose = targetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters,
      double angleToleranceDegrees) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtAngle = this.isAtTargetHeading(angleToleranceDegrees);
    boolean isAtTargetPose = isAtXAxis && isAtYAxis && isAtAngle;
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtXAxis", isAtXAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtYAxis", isAtYAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtTargetPose", isAtTargetPose);
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtXAxis", isAtXAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtYAxis", isAtYAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtTargetPose", isAtTargetPose);
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(Pose2d poseTarget, double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - poseTarget.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - poseTarget.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtXAxis", isAtXAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtYAxis", isAtYAxis);
    Logger.recordOutput("Subsystems/Swerve/Odometry/isAtTargetPose", isAtTargetPose);
    return isAtTargetPose;
  }

  @Override
  protected void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    super.driveFieldOriented(targetSpeeds);
  }

  @Override
  protected void driveRobotOriented(ChassisSpeeds targetSpeeds) {
    super.driveRobotOriented(targetSpeeds);
  }

  protected Command driveToPoseWithPathfinding(Pose2d targetPose) {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0);
    return pathfindingCommand;
  }

  protected Command driveToPathWithPathfinding(PathPlannerPath path) {
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
  }

  @Override
  public void periodic() {
    this.updateLimelightRequiredValues();
    this.updateOdometry();
    super.periodic();
    this.updateOdometrySwerveLogs();
  }

  private void updateLimelightRequiredValues() {
    OdometrySwerveSubsystem.robotAngularVelocity = this.getRobotVelocity().omega;
    OdometrySwerveSubsystem.robotOrientation = this.getState().Pose.getRotation().getDegrees();
  }

  private void updateOdometrySwerveLogs() {
    Logger.recordOutput("Subsystems/Swerve/Odometry/targetAimPose", targetAimPose);
    Logger.recordOutput("Subsystems/Swerve/Odometry/targetPose", targetPose);
  }
}
