package frc.Java_Is_UnderControl.Swerve;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;

public abstract class OdometryEnabledSwerveSubsystem extends BaseSwerveSubsystem {

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

  private CustomDoubleLogger targetVyDriveToPoseLogger = new CustomDoubleLogger("/SwerveSubsystem/targetVyDriveToPose");

  private CustomDoubleLogger targetVxDriveToPoseLogger = new CustomDoubleLogger("/SwerveSubsystem/targetVxDriveToPose");

  private CustomPose2dLogger targetPoseLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetPose");

  private CustomPose2dLogger targetAimPositionLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetAimPosition");

  private CustomBooleanLogger isAtTargetXAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetXAxis");

  private CustomBooleanLogger isAtTargetYAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetYAxis");

  private CustomBooleanLogger isAtTargetPoseLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetPose");

  private CustomPose2dLogger poseVisionLogger = new CustomPose2dLogger("/SwerveSubsystem/PoseVision");

  private CustomStringLogger poseEstimatorNameLogger = new CustomStringLogger("/SwerveSubsystem/PoseEstimatorName");

  // Create the constraints to use while pathfinding. The constraints defined in
  // the path will only be used for the path.
  PathConstraints constraints;

  public OdometryEnabledSwerveSubsystem(OdometryEnabledSwerveConfig config,
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

  public OdometryEnabledSwerveSubsystem(OdometryEnabledSwerveConfig config,
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
      this.poseVisionLogger.appendRadians(poseVision);
      this.setVisionStdDev(estimatedPose.visionStdDev);
      this.addVisionMeasurement(poseVision, estimatedPose.timestampSeconds);
      this.poseEstimatorNameLogger.append(poseEstimator.getEstimatorName());
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
    this.targetVxDriveToPoseLogger.append(targetXVelocity);
    this.targetVyDriveToPoseLogger.append(targetYVelocity);
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
    this.isAtTargetXAxisLogger.append(isAtXAxis);
    this.isAtTargetYAxisLogger.append(isAtYAxis);
    this.isAtTargetPoseLogger.append(isAtTargetPose);
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    this.isAtTargetXAxisLogger.append(isAtXAxis);
    this.isAtTargetYAxisLogger.append(isAtYAxis);
    this.isAtTargetPoseLogger.append(isAtTargetPose);
    return isAtTargetPose;
  }

  protected boolean isAtTargetPose(Pose2d poseTarget, double xAxisToleranceMeters, double yAxisToleranceMeters) {
    double xAxisDistance = Math.abs(this.getPose().getX() - poseTarget.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - poseTarget.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtTargetPose = isAtXAxis && isAtYAxis;
    this.isAtTargetXAxisLogger.append(isAtXAxis);
    this.isAtTargetYAxisLogger.append(isAtYAxis);
    this.isAtTargetPoseLogger.append(isAtTargetPose);
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

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(0.25);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(5);
                  this.driveRobotOriented(new ChassisSpeeds(0.0, 0.0, speed));
                  System.out.println(this.driveBaseRadius);
                },
                this)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.wait(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = this.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = this.getHeading();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = this.getHeading();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = super.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  protected class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = this.getState().ModuleStates[i].angle.getRadians();
    }
    return values;
  }

  @Override
  public void periodic() {
    this.updateLimelightRequiredValues();
    this.updateOdometry();
    super.periodic();
    this.updateOdometrySwerveLogs();
  }

  private void updateLimelightRequiredValues() {
    OdometryEnabledSwerveSubsystem.robotAngularVelocity = this.getRobotVelocity().omega;
    OdometryEnabledSwerveSubsystem.robotOrientation = this.getState().Pose.getRotation().getDegrees();
  }

  private void updateOdometrySwerveLogs() {
    this.targetAimPositionLogger.appendRadians(targetAimPose);
    this.targetPoseLogger.appendRadians(this.targetPose);
  }

}
