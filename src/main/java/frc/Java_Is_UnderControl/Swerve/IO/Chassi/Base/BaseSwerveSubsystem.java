package frc.Java_Is_UnderControl.Swerve.IO.Chassi.Base;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Java_Is_UnderControl.Swerve.Configs.BaseSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.Configs.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants.TunerSwerveDrivetrain;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CustomMath;
import frc.Java_Is_UnderControl.Util.Util;

public abstract class BaseSwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem, BaseSwerveSubsystemIO {

  public double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  protected double MaxAngularRate;
  protected final double driveBaseRadius = Math
      .hypot(SwerveConstants.FrontLeft.LocationX + SwerveConstants.FrontRight.LocationX / 2,
          SwerveConstants.FrontLeft.LocationY + SwerveConstants.BackLeft.LocationY / 2);

  private static final double kSimLoopPeriod = 0.005;
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final SwerveRequest.ApplyRobotSpeeds applyRobotCentricSpeeds = new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);
  private final SwervePathPlannerConfig pathPlannerConfig;

  private final SwerveRequest.FieldCentric applyFieldCentricDrive = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle applyFieldCentricDrivePointingAtAngle = new FieldCentricFacingAngle();
  private SwerveRequest.RobotCentric applyRobotCentricDrive = new RobotCentric();
  private SwerveRequest.SwerveDriveBrake applyBrakeSwerveX = new SwerveDriveBrake();

  private Matrix<N3, N1> actualVisionStdDev;

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private double targetHeadingDegrees = Double.NaN;

  private double lastDesiredJoystickAngle;

  private final BaseSwerveSubsystemIOInputsAutoLogged swerveInputs;

  private boolean isAtTargetHeading;

  protected BaseSwerveSubsystem(BaseSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    applyFieldCentricDrivePointingAtAngle.HeadingController = new PhoenixPIDController(config.headingPidConfig.kP,
        config.headingPidConfig.kI, config.headingPidConfig.kD);
    applyFieldCentricDrivePointingAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    MaxAngularRate = RotationsPerSecond.of(config.maxRotationRate).in(RadiansPerSecond);
  
    pathPlannerConfig = config.pathPlannerConfig;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    this.lastDesiredJoystickAngle = AllianceFlipUtil.shouldFlip() ? 0 : 180;
    this.swerveInputs = new BaseSwerveSubsystemIOInputsAutoLogged();
    this.isAtTargetHeading = false;
    this.actualVisionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);
  }

  protected BaseSwerveSubsystem(BaseSwerveConfig config, SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    applyFieldCentricDrivePointingAtAngle.HeadingController = new PhoenixPIDController(config.headingPidConfig.kP,
        config.headingPidConfig.kI, config.headingPidConfig.kD);
    applyFieldCentricDrivePointingAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    MaxAngularRate = RotationsPerSecond.of(config.maxRotationRate).in(RadiansPerSecond);
                                                                                        
    pathPlannerConfig = config.pathPlannerConfig;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();

    this.lastDesiredJoystickAngle = AllianceFlipUtil.shouldFlip() ? 0 : 180;
    this.swerveInputs = new BaseSwerveSubsystemIOInputsAutoLogged();
    this.isAtTargetHeading = false;
    this.actualVisionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);
  }

  @Override
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotVelocity,
          (speeds, feedforwards) -> setControl(

              applyRobotCentricSpeeds.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
                                        
              pathPlannerConfig.translationPid,
              pathPlannerConfig.anglePid
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  @Override
  public void resetOdometry(Pose2d initialHolonomicPose) {
    this.resetPose(initialHolonomicPose);
  }

  @Override
  public void resetTranslation(Translation2d translationToReset) {
    super.resetTranslation(translationToReset);
  }

  @Override
  public void zeroGyro() {
    super.getPigeon2().setYaw(0);
  }

  @Override
  public void setHeadingCorrection(boolean active) {}

  @Override
  public void setMotorBrake(boolean brake) {
    if (brake) {
      super.configNeutralMode(NeutralModeValue.Brake);
    } else {
      super.configNeutralMode(NeutralModeValue.Coast);
    }
  }

  protected Pose2d getPose() {
    return super.getState().Pose;
  }
    
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
        visionMeasurementStdDevs);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  protected void setVisionStdDev(Matrix<N3, N1> visionMeasurementsStdDev) {
    if (visionMeasurementsStdDev.isEqual(actualVisionStdDev, 0)) {
      return;
    }
    setVisionMeasurementStdDevs(visionMeasurementsStdDev);
    actualVisionStdDev = visionMeasurementsStdDev;
  }

  protected Pose2d getEarlyPoseMoving(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation = new Translation2d(
        actualPose.getX() + getFieldOrientedRobotVelocity().vx * dt,
        actualPose.getY() + getFieldOrientedRobotVelocity().vy * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, actualPose.getRotation());
    return earlyPoseMoving;
  }

  protected Pose2d getEarlyPoseMovingWithOmega(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation = new Translation2d(
        actualPose.getX() + getFieldOrientedRobotVelocity().vx * dt,
        actualPose.getY() + getFieldOrientedRobotVelocity().vy * dt);
    Rotation2d earlyRotation = new Rotation2d(
        actualPose.getRotation().getRadians() + getFieldOrientedRobotVelocity().omega * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, earlyRotation);
    return earlyPoseMoving;
  }

  protected Supplier<SwerveRequest> setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.targetSpeeds = chassisSpeeds;
    return () -> applyRobotCentricSpeeds.withSpeeds(chassisSpeeds);
  }

  protected ChassisSpeeds getRobotVelocity() {
    return super.getState().Speeds;
  }

  protected ChassisSpeeds getFieldOrientedRobotVelocity() {
    return super.getState().Speeds.toFieldRelative(getHeading());
  }

  protected double getAbsoluteRobotVelocity() {
    return CustomMath.toAbsoluteSpeed(this.getRobotVelocity());
  }

  protected Rotation2d getHeading() {
    return super.getState().Pose.getRotation();
  }

  protected double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = getState().ModulePositions[i].distance / SwerveConstants.WHEEL_RADIUS_METERS;
    }
    return values;
  }

  @Override
  public Supplier<SwerveRequest> lock() {
    return () -> applyBrakeSwerveX;
  }

  protected abstract void updateLogs();

  @Override
  public void periodic() {
    this.updateBaseLogs();
    this.updateLogs();
  }

  private void updateBaseLogs() {
    this.swerveInputs.stdDevXY = actualVisionStdDev.get(0, 0);
    this.swerveInputs.stdDevTheta = actualVisionStdDev.get(2, 0);
    this.swerveInputs.currentPose = this.getPose();
    this.swerveInputs.targetSpeeds = this.targetSpeeds;
    this.swerveInputs.absoluteTargetSpeed = CustomMath.toAbsoluteSpeed(this.targetSpeeds);
    this.swerveInputs.measuredSpeeds = this.getRobotVelocity();
    this.swerveInputs.absoluteMeasuredSpeed = this.getAbsoluteRobotVelocity();
    this.swerveInputs.targetHeadingDegrees = this.targetHeadingDegrees;
    this.swerveInputs.measuredHeadingDegrees = this.getHeading().getDegrees();
    this.swerveInputs.isAtTargetHeading = this.isAtTargetHeading;

    Logger.processInputs("Subsystems/Swerve/", this.swerveInputs);
  }

  protected void driveFieldOrientedLockedAngle(ChassisSpeeds speeds, Rotation2d targetHeading) {
    this.lastDesiredJoystickAngle = targetHeading.getRadians();
    this.targetHeadingDegrees = targetHeading.getDegrees();
    applyFieldCentricDrivePointingAtAngle.withTargetDirection(targetHeading)
        .withVelocityX(speeds.vx).withVelocityY(speeds.vy);
    setControl(applyFieldCentricDrivePointingAtAngle);
  }

  protected void driveFieldOrientedLockedJoystickAngle(ChassisSpeeds speeds, double xHeading, double yHeading) {
    double angle = Util.withinHypotDeadband(xHeading, yHeading) ? lastDesiredJoystickAngle
          : Math.atan2(xHeading, yHeading);
      this.targetHeadingDegrees = Units.radiansToDegrees(angle);
      this.lastDesiredJoystickAngle = Units.degreesToRadians(this.targetHeadingDegrees);
      applyFieldCentricDrivePointingAtAngle
          .withTargetDirection(Rotation2d.fromDegrees(targetHeadingDegrees))
          .withVelocityX(speeds.vx).withVelocityY(speeds.vy);

    setControl(applyFieldCentricDrivePointingAtAngle);
  }

  protected void driveFieldOriented(ChassisSpeeds speeds) {
    this.targetSpeeds = speeds;
    this.targetHeadingDegrees = Double.NaN;
    applyFieldCentricDrive.withVelocityX(speeds.vx).withVelocityY(speeds.vy)
        .withRotationalRate(speeds.omega);
    setControl(applyFieldCentricDrive);
  }

  protected void driveRobotOriented(ChassisSpeeds speeds) {
    this.targetSpeeds = speeds;
    this.targetHeadingDegrees = Double.NaN;
    applyRobotCentricDrive.withVelocityX(speeds.vx).withVelocityY(speeds.vy)
        .withRotationalRate(speeds.omega);
    setControl(applyRobotCentricDrive);
  }

  protected ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput, double AngularRate) {
    return new ChassisSpeeds(xInput * this.MaxSpeed, yInput * this.MaxSpeed, AngularRate * this.MaxAngularRate);
  }

  protected ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput) {
    return new ChassisSpeeds(xInput * this.MaxSpeed, yInput * this.MaxSpeed, 0);
  }

  protected boolean isAtTargetHeading(double toleranceDegrees) {
    if (this.targetHeadingDegrees == Double.NaN) {
      this.isAtTargetHeading = false;
      return false;
    }
    Rotation2d targetHeading = Rotation2d.fromDegrees(this.targetHeadingDegrees);
    Rotation2d currentHeading = this.getHeading();
    double headingDifferenceDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
    boolean isAtHeading = headingDifferenceDegrees <= toleranceDegrees;
    this.isAtTargetHeading = isAtHeading;
    return isAtHeading;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();


    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

  
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
