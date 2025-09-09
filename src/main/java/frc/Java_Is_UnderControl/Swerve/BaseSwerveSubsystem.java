package frc.Java_Is_UnderControl.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Swerve.Configs.BaseSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.IO.Gyro.GyroIOPigeon2;
import frc.Java_Is_UnderControl.Util.CustomMath;
import frc.robot.util.LocalADStarAK;

import java.util.HashMap;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class BaseSwerveSubsystem implements Subsystem {

  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();
  private Rotation2d robotAngle;
  private Pose2d robotPose = new Pose2d();

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
  private final GyroIOPigeon2 pigeon;
  private final BaseSwerveConfig config;
  private final PIDController headingPidController;

  public final double maxSpeed;
  public final  double maxAngularRate;

  private final SwerveChassisTrancription chassisTrancription;

  private boolean isUsingRobotCenter = true;

  private Translation2d centerOfRotation;

  private double targetHeadingDegrees = 0;

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          SwerveConstants.ROBOT_MASS,
          SwerveConstants.ROBOT_MOI,
          new ModuleConfig(
              SwerveConstants.WHEEL_RADIUS_METERS,
              SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              SwerveConstants.WHEEL_COF,
              DCMotor.getKrakenX60Foc(2).withReduction(SwerveConstants.GEARBOX_REDUCTION),
              SwerveConstants.FrontLeft.SlipCurrent,
              2),
          SwerveConstants.MODULE_OFFSETS);

  private final HashMap<
          String,
          Optional<
              SwerveModuleConstants<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>>
      motorConstants;

  private final Drive drive;

  public BaseSwerveSubsystem(
      BaseSwerveConfig config,
      HashMap<
              String,
              SwerveModuleConstants<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
          moduleConstants) {
    this.maxAngularRate =
        RotationsPerSecond.of(config.maxRotationRateInFractions)
            .in(RadiansPerSecond); // fraction of a rotation per second
    this.chassisTrancription = new SwerveChassisTrancription();
    this.config = config;
    this.headingPidController = new PIDController(config.headingPidConfig.kP, config.headingPidConfig.kI, config.headingPidConfig.kD);
    configureAutoBuilder();
    this.maxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    this.motorConstants = new HashMap<>();
    this.motorConstants.put(
        SwerveConstants.FRONT_LEFT_MODULE_NAME,
        Optional.of(moduleConstants.get(SwerveConstants.FRONT_LEFT_MODULE_NAME)));
    this.motorConstants.put(
        SwerveConstants.FRONT_RIGHT_MODULE_NAME,
        Optional.of(moduleConstants.get(SwerveConstants.FRONT_RIGHT_MODULE_NAME)));
    this.motorConstants.put(
        SwerveConstants.BACK_LEFT_MODULE_NAME,
        Optional.of(moduleConstants.get(SwerveConstants.BACK_LEFT_MODULE_NAME)));
    this.motorConstants.put(
        SwerveConstants.BACK_RIGHT_MODULE_NAME,
        Optional.of(moduleConstants.get(SwerveConstants.BACK_RIGHT_MODULE_NAME)));
    this.pigeon = new GyroIOPigeon2();
    this.drive = new Drive(this.motorConstants, pigeon);
    this.robotAngle = Rotation2d.fromDegrees(this.pigeon.getYaw());
  }

  // Tem na documentação do advantagekit de como fazer
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return null;
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return null;
  }

  // tem na documentação
  private void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getRobotPose,
        this::setPose,
        this::getRobotSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  private Pose2d getRobotPose() {
    return this.robotPose;
  }

  private ChassisSpeeds getRobotSpeeds() {
    return this.robotSpeeds;
  }

  public ChassisSpeeds getFieldOrientedRobotSpeeds() {
    return null;
  }

  public void setHeadingCorrection(boolean active) {}

  private void setPose(Pose2d pose) {}

  public void resetOdometry(Pose2d initialHolonomicPose) {}

  public void resetTranslation(Pose2d translationToReset) {}

  private void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {}

  public double getAbsoluteRobotVelocity() {
    return CustomMath.toAbsoluteSpeed(this.getRobotSpeeds());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = 0;
    }
    return values;
  }

  @Override
  public void periodic() {
    if (this.isUsingRobotCenter) {
      this.chassisTrancription.setChassisSpeeds(this.targetSpeeds);
    } else {
      this.chassisTrancription.setChassisSpeedsWithDifferentCenterOfRotation(
          this.targetSpeeds, centerOfRotation);
    }

    this.drive.updateModuleTargetStates(this.chassisTrancription.getModuleInputs());
    this.drive.updateLogs();
    this.robotSpeeds = this.drive.getRobotSpeeds();
    this.robotAngle = Rotation2d.fromDegrees(this.drive.getRobotAngle());

    this.updateSwerveLogs();
  }

  public void driveFieldOrientedLockedAngle(ChassisSpeeds speeds, double xHeading, double yHeading) {
    double angularVelocity = this.headingPidController.calculate(
      Units.radiansToDegrees(this.robotAngle.getRadians()),
      Units.radiansToDegrees(Math.atan2(yHeading, xHeading))
    );

    ChassisSpeeds finalSpeed = new ChassisSpeeds(speeds.vx, speeds.vy, angularVelocity);

    this.targetSpeeds =
        finalSpeed.toFieldRelative(this.robotAngle);
  }

  public void driveFieldOriented(ChassisSpeeds speeds) {
    this.targetSpeeds =
        speeds.toRobotRelative(this.robotAngle);
  }

  public void driveRobotOriented(ChassisSpeeds speeds) {
    this.targetSpeeds =
        speeds.toRobotRelative(this.robotAngle);
  }

  public ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput, double AngularRate) {
    return new ChassisSpeeds(
        xInput * this.maxSpeed, yInput * this.maxSpeed, AngularRate * this.maxAngularRate);
  }

  public ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput) {
    return new ChassisSpeeds(xInput * this.maxSpeed, yInput * this.maxSpeed, 3);
  }

  public boolean isAtTargetHeading(double toleranceDegrees) {
    if (this.targetHeadingDegrees == Double.NaN) {
      return false;
    }
    Rotation2d targetHeading = Rotation2d.fromDegrees(this.targetHeadingDegrees);
    Rotation2d currentHeading = this.robotAngle;
    double headingDifferenceDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
    boolean isAtHeading = headingDifferenceDegrees <= toleranceDegrees;
    return isAtHeading;
  }

  public void updateSwerveLogs(){
    Logger.recordOutput("/Swerve/Subsystems/Robot Angle", this.robotAngle.getDegrees());
    Logger.recordOutput("/Swerve/Subsystems/Robot Speeds", this.robotSpeeds);
    Logger.recordOutput("/Swerve/Subsystems/Target Speeds", this.targetSpeeds);
    Logger.recordOutput("/Subsystems/Swerve/Module States", this.drive.getRobotModuleStates());
    Logger.recordOutput("/Subsystems/Swerve/Module Target States", this.drive.getRobotModuleTargetStates());
  }
}
