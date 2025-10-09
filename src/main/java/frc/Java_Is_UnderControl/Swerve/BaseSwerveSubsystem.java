package frc.Java_Is_UnderControl.Swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomChassisSpeedsLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants.TunerSwerveDrivetrain;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CustomMath;
import frc.Java_Is_UnderControl.Util.Util;

public abstract class BaseSwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  public double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  protected double MaxAngularRate;
  protected final double driveBaseRadius = Math
      .hypot(SwerveConstants.FrontLeft.LocationX + SwerveConstants.FrontRight.LocationX / 2,
          SwerveConstants.FrontLeft.LocationY + SwerveConstants.BackLeft.LocationY / 2);

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyRobotSpeeds applyRobotCentricSpeeds = new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity);
  private final SwervePathPlannerConfig pathPlannerConfig;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric applyFieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.PointWheelsAt applyPointWheelsAt = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.FieldCentricFacingAngle applyFieldCentricDrivePointingAtAngle = new FieldCentricFacingAngle();
  private SwerveRequest.RobotCentric applyRobotCentricDrive = new RobotCentric();
  private SwerveRequest.SwerveDriveBrake applyBrakeSwerveX = new SwerveDriveBrake();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  private Matrix<N3, N1> actualVisionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private double targetHeadingDegrees = Double.NaN;

  private double lastDesiredJoystickAngle;

  private CustomChassisSpeedsLogger targetSpeedsLogger = new CustomChassisSpeedsLogger("/SwerveSubsystem/TargetSpeeds");

  private CustomDoubleLogger absoluteTargetSpeedLogger = new CustomDoubleLogger("/SwerveSubsystem/AbsoluteTargetSpeed");

  private CustomChassisSpeedsLogger measuredSpeedsLogger = new CustomChassisSpeedsLogger(
      "/SwerveSubsystem/MeasuredSpeeds");

  private CustomDoubleLogger absoluteMeasuredSpeedLogger = new CustomDoubleLogger(
      "/SwerveSubsystem/AbsoluteMeasuredSpeed");

  private CustomDoubleLogger stdDevXYLogger = new CustomDoubleLogger("/SwerveSubsystem/XyStdDev");

  private CustomDoubleLogger stdDevThetaLogger = new CustomDoubleLogger("/SwerveSubsystem/ThetaStdDev");

  private CustomPose2dLogger poseLogger = new CustomPose2dLogger("/SwerveSubsystem/Pose");

  private CustomDoubleLogger targetHeadingLogger = new CustomDoubleLogger("/SwerveSubsystem/TargetHeadingDegrees");

  private CustomDoubleLogger measuredHeadingLogger = new CustomDoubleLogger("/SwerveSubsystem/MeasuredHeadingDegrees");

  private CustomBooleanLogger isAtTargetHeading = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetHeading");

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

  protected BaseSwerveSubsystem(BaseSwerveConfig config,
      SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    applyFieldCentricDrivePointingAtAngle.HeadingController = new PhoenixPIDController(config.headingPidConfig.kP,
        config.headingPidConfig.kI, config.headingPidConfig.kD);
    applyFieldCentricDrivePointingAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    MaxAngularRate = RotationsPerSecond.of(config.maxRotationRate).in(RadiansPerSecond); // fraction of a rotation per
    // second
    pathPlannerConfig = config.pathPlannerConfig;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    this.lastDesiredJoystickAngle = AllianceFlipUtil.shouldFlip() ? 0 : 180;
  }

  protected BaseSwerveSubsystem(BaseSwerveConfig config, SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    applyFieldCentricDrivePointingAtAngle.HeadingController = new PhoenixPIDController(config.headingPidConfig.kP,
        config.headingPidConfig.kI, config.headingPidConfig.kD);
    applyFieldCentricDrivePointingAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    MaxAngularRate = RotationsPerSecond.of(config.maxRotationRate).in(RadiansPerSecond); // fraction of a rotation per
                                                                                         // second
    pathPlannerConfig = config.pathPlannerConfig;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();

    this.lastDesiredJoystickAngle = AllianceFlipUtil.shouldFlip() ? 0 : 180;
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> setControl(

              applyRobotCentricSpeeds.withSpeeds(speeds)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              pathPlannerConfig.translationPid, // Translation PID constants
              pathPlannerConfig.anglePid // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    this.resetPose(initialHolonomicPose);
  }

  public void resetTranslation(Translation2d translationToReset) {
    super.resetTranslation(translationToReset);
  }

  public void zeroGyro() {
    super.getPigeon2().setYaw(0);
  }

  public void setHeadingCorrection(boolean active) {
    // ;
  }

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

  public Supplier<SwerveRequest> lock() {
    return () -> applyBrakeSwerveX;
  }

  protected abstract void updateLogs();

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    // DriverStation.getAlliance().ifPresent(allianceColor -> {
    // setOperatorPerspectiveForward(
    // allianceColor == Alliance.Red
    // ? kRedAlliancePerspectiveRotation
    // : kBlueAlliancePerspectiveRotation);
    // m_hasAppliedOperatorPerspective = true;
    // });
    // }
    this.updateBaseLogs();
    this.updateLogs();
    Logger.recordOutput("TargetHeading", this.targetHeadingDegrees);
  }

  private void updateBaseLogs() {
    this.stdDevXYLogger.append(actualVisionStdDev.get(0, 0));
    this.stdDevThetaLogger.append(actualVisionStdDev.get(2, 0));
    this.poseLogger.appendRadians(this.getPose());
    this.targetSpeedsLogger.append(this.targetSpeeds);
    this.absoluteTargetSpeedLogger.append(CustomMath.toAbsoluteSpeed(this.targetSpeeds));
    this.measuredSpeedsLogger.append(this.getRobotVelocity());
    this.absoluteMeasuredSpeedLogger.append(this.getAbsoluteRobotVelocity());
    this.targetHeadingLogger.append(this.targetHeadingDegrees);
    this.measuredHeadingLogger.append(this.getHeading().getDegrees());
    this.poseLogger.appendRadians(this.getPose());
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
      this.isAtTargetHeading.append(false);
      return false;
    }
    Rotation2d targetHeading = Rotation2d.fromDegrees(this.targetHeadingDegrees);
    Rotation2d currentHeading = this.getHeading();
    double headingDifferenceDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
    boolean isAtHeading = headingDifferenceDegrees <= toleranceDegrees;
    this.isAtTargetHeading.append(isAtHeading);
    return isAtHeading;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
