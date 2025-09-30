package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Swerve.MoveToPosePIDConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.Java_Is_UnderControl.Vision.Odometry.LimelightPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.MultiCameraPoseEstimator;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.AutoAlignConstants;
import frc.robot.constants.SwerveConstants.PoseEstimatorState;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.SwerveConstants.TargetFace;
import frc.robot.joysticks.DriverController;
import frc.robot.pose_estimators.ReefPoseEstimatorWithLimelight;

public class SwerveSubsystem extends OdometryEnabledSwerveSubsystem implements ISwerve {

  private DriverController controller = DriverController.getInstance();

  private TargetBranch targetBranch = TargetBranch.A;

  private double goToPoseTranslationDeadband = 0.03;

  private double goToPoseHeadingDeadband = 1;

  Supplier<ReefLevel> scorerTargetReefLevelSupplier;

  Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgaeSupplier;

  int[] apriltagsIDs = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };

  private ReefPoseEstimatorWithLimelight reefPoseEstimator = new ReefPoseEstimatorWithLimelight("limelight-ggg",
      "limelight-right", () -> getTargetBranch());

  CustomStringLogger swerveStateLogger = new CustomStringLogger("SwerveSubsystem/State");

  CustomBooleanLogger forcingReefPoseEstimatorLogger = new CustomBooleanLogger(
      "SwerveSubsystem/forcingReefPoseEstimatorLogger");

  private StabilizeChecker stableAtTargetPose = new StabilizeChecker(0.150);

  private String state = "NULL";

  CustomStringLogger poseEstimatorStateLogger = new CustomStringLogger("SwerveSubsystem/State_PoseEstimator");

  CustomStringLogger targetBranchLogger = new CustomStringLogger("SwerveSubsystem/Target Branch");

  CustomDoubleLogger targetVelocity = new CustomDoubleLogger("SwerveSubsystem/TargetVelocity");

  CustomDoubleLogger distanceToTargetBranchLog = new CustomDoubleLogger("SwerveSubsystem/DistanceToTargetBranch");

  CustomDoubleLogger distanceToTargetFaceLog = new CustomDoubleLogger("SwerveSubsystem/DistanceToTargetFace");

  private Rotation2d bestAngleForClimb = new Rotation2d();

  private double distanceToTargetBranch = Double.POSITIVE_INFINITY;

  private double distanceToTargetFace = Double.POSITIVE_INFINITY;

  private boolean positionUpdated = false;

  private GoToBranchConfiguration goToBranchConfigurationFast;

  private GoToFaceConfiguration removeAlgaeFromBranchTeleoperatedConfiguration;

  private GoToFaceConfiguration removeAlgaeFromBranchAutonomousConfiguration;

  private GoToBranchConfiguration goToBranchConfigurationTeleoperated;

  private GoToFace goToFaceTeleoperatedSetpointDefiner;

  private GoToFace goToFaceAutonomousSetpointDefiner;

  private GoToBranch goToBranchTeleoperatedSetpointDefiner;

  private GoToBranch goToBranchFastSetpointDefiner;

  private Pose2d targetFacePose = TargetFace.A.getTargetPoseToScore();

  private static final SwervePathPlannerConfig pathPlannerConfig = new SwervePathPlannerConfig(
      new PIDConstants(5, 0, 0),
      new PIDConstants(5, 0, 0),
      new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720)));

  public SwerveSubsystem(Supplier<ReefLevel> scorerTargetReefLevel,
      Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgae,
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(new OdometryEnabledSwerveConfig(0.75, pathPlannerConfig,
        new LimelightPoseEstimator("limelight-ggg", false, false, 2),
        new LimelightPoseEstimator("limelight-ggg", false, false, 2),
        new PIDConfig(6, 0, 0),
        new MoveToPosePIDConfig(SwerveConstants.MOVE_TO_POSE_TRANSLATION_PID,
            SwerveConstants.MOVE_TO_POSE_Y_CONSTRAINTS)),
        drivetrainConstants,
        modules);
    this.scorerTargetReefLevelSupplier = scorerTargetReefLevel;
    this.scorerTargetReefLevelAlgaeSupplier = scorerTargetReefLevelAlgae;
    this.configureGoToObjects();
    this.getPigeon2().setYaw(0);
    this.resetOdometry(new Pose2d());
  }

  private void configureGoToObjects() {
    this.removeAlgaeFromBranchTeleoperatedConfiguration = new GoToFaceConfiguration(
        AutoAlignConstants.PoseDeadBand.RemoveAlgae.MIN_ERROR_AUTO_ALIGN_FAST,
        AutoAlignConstants.PoseDeadBand.RemoveAlgae.MAX_ERROR_AUTO_ALIGN_FAST, "REMOVE_ALGAE",
        AutoAlignConstants.VelocitiesRelatedToDistance.RemoveAlgae.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.RemoveAlgae.MAX_VELOCITY_POSITION);
    this.removeAlgaeFromBranchAutonomousConfiguration = new GoToFaceConfiguration(
        AutoAlignConstants.PoseDeadBand.Teleoperated.MIN_ERROR_AUTO_ALIGN_TELEOPERATED,
        AutoAlignConstants.PoseDeadBand.Teleoperated.MAX_ERROR_AUTO_ALIGN_TELEOPERATED, "REMOVE_ALGAE_AUTONOMOUS",
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MAX_VELOCITY_POSITION);
    this.goToBranchConfigurationFast = new GoToBranchConfiguration(
        AutoAlignConstants.PoseDeadBand.Fast.MIN_ERROR_AUTO_ALIGN_FAST,
        AutoAlignConstants.PoseDeadBand.Fast.MAX_ERROR_AUTO_ALIGN_FAST, "FAST",
        AutoAlignConstants.VelocitiesRelatedToDistance.Fast.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.Fast.MAX_VELOCITY_POSITION, true);
    this.goToBranchConfigurationTeleoperated = new GoToBranchConfiguration(
        AutoAlignConstants.PoseDeadBand.Teleoperated.MIN_ERROR_AUTO_ALIGN_TELEOPERATED,
        AutoAlignConstants.PoseDeadBand.Teleoperated.MAX_ERROR_AUTO_ALIGN_TELEOPERATED, "TELEOPERATED",
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MIN_VELOCITY_POSITION,
        AutoAlignConstants.VelocitiesRelatedToDistance.Teleoperated.MAX_VELOCITY_POSITION, true);

    this.goToFaceTeleoperatedSetpointDefiner = new GoToFace(removeAlgaeFromBranchTeleoperatedConfiguration);
    this.goToFaceAutonomousSetpointDefiner = new GoToFace(removeAlgaeFromBranchTeleoperatedConfiguration);
    this.goToBranchFastSetpointDefiner = new GoToBranch(goToBranchConfigurationFast);
    this.goToBranchTeleoperatedSetpointDefiner = new GoToBranch(goToBranchConfigurationTeleoperated);
  }

  public void resetOdometryLimelight(Translation2d defaultPosition) {
    PoseEstimator limelightLeft = new LimelightPoseEstimator("limelight-ggg", false, false, 2);
    Optional<PoseEstimation> limelightPoseEstimation = limelightLeft.getEstimatedPose(this.getPose());
    if (limelightPoseEstimation.isEmpty()) {
      resetTranslation(defaultPosition);
      this.positionUpdated = true;
      System.out.println("POSE DA LIME É NULA");
    } else {
      System.out.println("USANDO A POSE DA LIME");
      resetTranslation(limelightPoseEstimation.get().estimatedPose.getTranslation().toTranslation2d());
      this.positionUpdated = true;
    }
  }

  public boolean positionUpdated() {
    return this.positionUpdated;
  }

  @Override
  public void driveAlignAngleJoystick() {
    if (controller.leftBumper().getAsBoolean()) {
      driveRotating(false);
      return;
    }
    if (controller.rightBumper().getAsBoolean()) {
      driveRotating(true);
      return;
    }
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  public void driveRotating(boolean rotateRight) {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation(), rotateRight ? -1.5 : 1.5);
    this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
    this.driveFieldOriented(desiredSpeeds);
  }

  public void setTargetBranch(TargetBranch targetBranch) {
    this.targetBranch = targetBranch;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("BACKUP NECESSARY", checkBackupNecessary());
    super.periodic();
    updateLogs();
    LimelightHelpers.SetRobotOrientation("limelight-ggg",
        OdometryEnabledSwerveSubsystem.robotOrientation,
        OdometryEnabledSwerveSubsystem.robotAngularVelocity, 0, 0, 0, 0);
  }

  protected void updateLogs() {
    this.swerveStateLogger.append(this.state);
    this.targetBranchLogger.append(this.targetBranch.name());
    Logger.recordOutput("SwerveModulesStates", getState().ModuleStates);
  }

  @Override
  public void driveToBranch(TargetBranch branch, boolean backup, boolean goDirect) {
    this.targetBranch = branch;
    if (DriverStation.isAutonomousEnabled()) {
      driveToBranchFast(branch, backup, goDirect);
      return;
    }
    goToBranchTeleoperated(branch, backup, goDirect);
  }

  private void driveToBranchFast(TargetBranch branch, boolean backup, boolean goDirect) {
    this.goToBranchFastSetpointDefiner.setBranch(branch, goDirect);
    this.goToBranchFastSetpointDefiner.updateBranchData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier, backup);
    this.distanceToTargetBranch = goToBranchFastSetpointDefiner.getDistanceToTargetBranch();
    this.targetVelocity.append(goToBranchFastSetpointDefiner.getFinalVelocity());
    this.distanceToTargetBranchLog.append(distanceToTargetBranch);
    driveToPose(this.goToBranchFastSetpointDefiner.getFinalPose(),
        this.goToBranchFastSetpointDefiner.getFinalVelocity());
    this.state = this.goToBranchFastSetpointDefiner.getGoToState();
  }

  private void goToBranchTeleoperated(TargetBranch branch, boolean backup, boolean goDirect) {
    this.goToBranchTeleoperatedSetpointDefiner.setBranch(branch, goDirect);
    this.goToBranchTeleoperatedSetpointDefiner.updateBranchData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier, backup);
    this.distanceToTargetBranch = goToBranchTeleoperatedSetpointDefiner.getDistanceToTargetBranch();
    this.targetVelocity.append(goToBranchTeleoperatedSetpointDefiner.getFinalVelocity());
    this.distanceToTargetBranchLog.append(distanceToTargetBranch);
    if (this.goToBranchTeleoperatedSetpointDefiner.getDistanceToTargetBranch() < 3.5) {
      driveToPose(this.goToBranchTeleoperatedSetpointDefiner.getFinalPose(),
          this.goToBranchTeleoperatedSetpointDefiner.getFinalVelocity());
      this.state = this.goToBranchTeleoperatedSetpointDefiner.getGoToState();
    } else {
      driveAlignAngleJoystick();
    }
  }

  @Override
  public void goToFaceTeleoperated(TargetBranch branch) {
    this.goToFaceTeleoperatedSetpointDefiner.setBranch(branch, true);
    this.goToFaceTeleoperatedSetpointDefiner.updateFaceData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier, true);
    this.distanceToTargetFace = goToFaceTeleoperatedSetpointDefiner.getDistanceToTargetFace();
    this.targetVelocity.append(goToFaceTeleoperatedSetpointDefiner.getFinalVelocity());
    this.distanceToTargetFaceLog.append(distanceToTargetFace);
    driveToPose(this.goToFaceTeleoperatedSetpointDefiner.getFinalPose(),
        this.goToFaceTeleoperatedSetpointDefiner.getFinalVelocity());
    this.targetFacePose = this.goToFaceTeleoperatedSetpointDefiner.getFinalPose();
    this.state = this.goToFaceTeleoperatedSetpointDefiner.getGoToState();
  }

  @Override
  public void goToFaceAutonomous(TargetBranch branch) {
    this.goToFaceAutonomousSetpointDefiner.setBranch(branch, true);
    this.goToFaceAutonomousSetpointDefiner.updateFaceData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier, true);
    this.distanceToTargetFace = goToFaceAutonomousSetpointDefiner.getDistanceToTargetFace();
    this.targetVelocity.append(goToFaceAutonomousSetpointDefiner.getFinalVelocity());
    this.distanceToTargetFaceLog.append(distanceToTargetFace);
    driveToPose(this.goToFaceAutonomousSetpointDefiner.getFinalPose(),
        this.goToFaceAutonomousSetpointDefiner.getFinalVelocity());
    this.targetFacePose = this.goToFaceAutonomousSetpointDefiner.getFinalPose();
    this.state = this.goToFaceAutonomousSetpointDefiner.getGoToState();
  }

  @Override
  public void goToCollectAlgaeFromFacePosition(TargetBranch branch) {
    this.goToFaceTeleoperatedSetpointDefiner.setBranch(branch, true);
    this.goToFaceTeleoperatedSetpointDefiner.updateFaceData(getPose(), scorerTargetReefLevelSupplier,
        scorerTargetReefLevelAlgaeSupplier, true);
    this.distanceToTargetFace = goToFaceTeleoperatedSetpointDefiner.getDistanceToTargetFace();
    this.targetVelocity.append(goToFaceTeleoperatedSetpointDefiner.getFinalVelocity());
    this.distanceToTargetFaceLog.append(distanceToTargetFace);
    driveToPose(this.goToFaceTeleoperatedSetpointDefiner.getFinalPose(),
        this.goToFaceTeleoperatedSetpointDefiner.getFinalVelocity());
    this.targetFacePose = this.goToFaceTeleoperatedSetpointDefiner.getFinalPose();
    this.state = this.goToFaceTeleoperatedSetpointDefiner.getGoToState();
  }

  @Override
  public boolean isAtTargetFacePositionWithoutHeading() {
    return stableAtTargetPose
        .isStableInCondition(() -> isAtTargetPose(this.targetFacePose, this.goToPoseTranslationDeadband,
            this.goToPoseTranslationDeadband));
  }

  @Override
  public boolean checkBackupNecessary() {
    return getPose().getTranslation().getDistance(targetBranch.getTargetPoseToScore().getTranslation()) < 1;
  }

  @Override
  public boolean checkPivotWillCrashOnReef() {
    return getPose().getTranslation().getDistance(targetBranch.getTargetPoseToScore().getTranslation()) < 0.2;
  }

  @Override
  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

  private TargetBranch getTargetBranch() {
    return this.targetBranch;
  }

  private Pose2d getNearestCoralStationPose() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      if (this.getPose().getY() >= 4.0259) {
        return SwerveConstants.CoralStations.RedAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      } else {
        return SwerveConstants.CoralStations.RedAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      }
    } else {
      if (this.getPose().getY() >= 4.0259) {
        return SwerveConstants.CoralStations.BlueAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      } else {
        return SwerveConstants.CoralStations.BlueAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      }
    }
  }

  public void setAngleForClimb() {
    if (this.getPose().getX() >= FieldConstants.fieldLength / 2) {
      this.bestAngleForClimb = Rotation2d.fromDegrees(-90);
    } else {
      this.bestAngleForClimb = Rotation2d.fromDegrees(90);
    }
  }

  @Override
  public void driveLockedAngleToNearestCoralStation() {
    if (controller.leftBumper().getAsBoolean()) {
      driveRotating(false);
      return;
    }
    if (controller.rightBumper().getAsBoolean()) {
      driveRotating(true);
      return;
    }
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
      return;
    }
    Rotation2d nearestCoralStationRotationAngle = this.getNearestCoralStationPose().getRotation();
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_CORAL_STATION";
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
    } else {
      this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, nearestCoralStationRotationAngle.getCos(),
          nearestCoralStationRotationAngle.getSin());
    }
  }

  @Override
  public void driveLockedAngleToClimb() {
    if (!controller.notUsingJoystick()) {
      this.driveAlignAngleJoystick();
      return;
    }
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation());
    this.state = "DRIVE_ALIGN_ANGLE_CLIMB";
    this.driveFieldOrientedLockedAngle(desiredSpeeds.times(0.5), this.bestAngleForClimb);
  }

  @Override
  public void driveToNearestCoralStation() {
    Pose2d nearestCoralStationPose2D = this.getNearestCoralStationPose();
    this.driveToPose(nearestCoralStationPose2D);
  }

  @Override
  public boolean isAtTargetPositionWithHeading() {
    return stableAtTargetPose.isStableInCondition(() -> isAtTargetPose(this.goToPoseTranslationDeadband,
        this.goToPoseTranslationDeadband, this.goToPoseHeadingDeadband));
  }

  @Override
  public boolean isAtTargetPositionWithoutHeading() {
    return stableAtTargetPose.isStableInCondition(() -> isAtTargetPose(this.goToPoseTranslationDeadband,
        this.goToPoseTranslationDeadband));
  }

  public void driveToPoseTest() {
    driveToPose(new Pose2d(15, 2, new Rotation2d(Units.degreesToRadians(180))));
  }

  @Override
  public void stopSwerve() {
    this.driveRobotOriented(new ChassisSpeeds());
    this.state = "STOP_SWERVE";
  }

  @Override
  public void driveAlignAngleJoystickSuperSlow() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation()).times(0.12);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_SUPER_SLOW";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  public void driveAlignAngleJoystickRemoveAlgae() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(),
        controller.getXtranslation()).times(0.5);
    this.state = "DRIVE_ALIGN_ANGLE_JOY_REMOVE_ALGAE";
    this.driveFieldOrientedLockedJoystickAngle(desiredSpeeds, controller.getCOS_Joystick(),
        controller.getSIN_Joystick());
  }

  @Override
  public boolean swerveIsToCloseToReefForLiftingElevador() {
    return getPose().getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center)) < 1;
  }

}