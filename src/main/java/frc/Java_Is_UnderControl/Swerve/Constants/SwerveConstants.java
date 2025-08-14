package frc.Java_Is_UnderControl.Swerve.Constants;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import frc.Java_Is_UnderControl.Util.JsonUtil;

public final class SwerveConstants {
  private static final String JSON = "swerveConfig.json";

  public static final String FRONT_LEFT_MODULE_NAME = "Front Left Module";
  public static final String FRONT_RIGHT_MODULE_NAME = "Front Right Module";
  public static final String BACK_LEFT_MODULE_NAME = "Back Left Module";
  public static final String BACK_RIGHT_MODULE_NAME = "Back Right Module";

  public static final CANBus CAN_BUS = new CANBus(
    JsonUtil.getString("/Swerve/canBus", JSON),
    "./logs/example.hoot");

  public static final SwerveDrivetrainConstants DrivetrainConstants =
    new SwerveDrivetrainConstants()
      .withCANBusName(CAN_BUS.getName())
      .withPigeon2Id(JsonUtil.getInt("/Modules/pigeonID", JSON))
      .withPigeon2Configs(null);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
    createModuleConstants("Front Left Module", true);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
    createModuleConstants("Front Right Module", false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
    createModuleConstants("Back Left Module", true);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
    createModuleConstants("Back Right Module", false);

  public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getModuleConstants() {
    @SuppressWarnings("unchecked")
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules = 
      new SwerveModuleConstants[] { FrontLeft, FrontRight, BackLeft, BackRight };
    return modules;
  }

  public static final double ROBOT_MASS = JsonUtil.getDouble("/Swerve/massKilograms", JSON);
  public static final double ROBOT_MOMENT_OF_INERTIA = JsonUtil.getDouble("/Swerve/momentOfInertia", JSON);
  public static final double WHEEL_RADIUS_METERS = JsonUtil.getDouble("/Swerve/wheelRadiusMeters", JSON);
  public static final double WHEEL_COEFICIENT_OF_FRICTION = JsonUtil.getDouble("/Swerve/wheelCoeficient", JSON);
  public static final double ROBOT_SPEED_12_VOLTS = (JsonUtil.getDouble("/Swerve/speedAt12Volts", JSON));
  public static final double SLIP_CURRENT = JsonUtil.getDouble("/Swerve/slipCurrent", JSON);
  public static final double DRIVE_GEARBOX_RATIO = JsonUtil.getDouble("/Swerve/driveRatio", JSON);

  public static final Lock STOPPER_ODOMETRY_UPDATES_WHILE_READING_DATA = new ReentrantLock();
  public static final double HIGH_FREQUENCY = 250;
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double LOW_FREQUENCY = 50;
  public static final int MAX_NUMBER_CONFIG_ATTEMPTS = 5;
  public static final double TIMEOUT_SECONDS_CONFIG = 0.25;
  

  private static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(JsonUtil.getDouble("/Gains/Steer/kP", JSON))
    .withKI(JsonUtil.getDouble("/Gains/Steer/kI", JSON))
    .withKD(JsonUtil.getDouble("/Gains/Steer/kD", JSON))
    .withKS(JsonUtil.getDouble("/Gains/Steer/kS", JSON))
    .withKV(JsonUtil.getDouble("/Gains/Steer/kV", JSON))
    .withKA(JsonUtil.getDouble("/Gains/Steer/kA", JSON))
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(JsonUtil.getDouble("/Gains/Drive/kP", JSON))
    .withKI(JsonUtil.getDouble("/Gains/Drive/kI", JSON))
    .withKD(JsonUtil.getDouble("/Gains/Drive/kD", JSON))
    .withKS(JsonUtil.getDouble("/Gains/Drive/kS", JSON))
    .withKV(JsonUtil.getDouble("/Gains/Drive/kV", JSON))
    .withKA(JsonUtil.getDouble("/Gains/Drive/kA", JSON));

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
      .withDriveMotorGearRatio(DRIVE_GEARBOX_RATIO)
      .withSteerMotorGearRatio(JsonUtil.getDouble("/Swerve/steerRatio", JSON))
      .withCouplingGearRatio(JsonUtil.getDouble("/Swerve/coupleRatio", JSON))
      .withWheelRadius(Meters.of(WHEEL_RADIUS_METERS))
      .withSteerMotorGains(steerGains)
      .withDriveMotorGains(driveGains)
      .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
      .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
      .withSlipCurrent(Amps.of(SLIP_CURRENT))
      .withSpeedAt12Volts(MetersPerSecond.of(ROBOT_SPEED_12_VOLTS))
      .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
      .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
      .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
      .withDriveMotorInitialConfigs(new TalonFXConfiguration())
      .withSteerMotorInitialConfigs(new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(JsonUtil.getDouble("/Swerve/statorCurrentLimit", JSON)))
          .withStatorCurrentLimitEnable(true)))
      .withEncoderInitialConfigs(new CANcoderConfiguration())
      .withSteerInertia(KilogramSquareMeters.of(JsonUtil.getDouble("/Swerve/steerInertia", JSON)))
      .withDriveInertia(KilogramSquareMeters.of(JsonUtil.getDouble("/Swerve/driveInertia", JSON)))
      .withSteerFrictionVoltage(Volts.of(JsonUtil.getDouble("/Swerve/steerFrictionVoltage", JSON)))
      .withDriveFrictionVoltage(Volts.of(JsonUtil.getDouble("/Swerve/driveFrictionVoltage", JSON)));

  private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(
      String moduleName, boolean invertSide) {
    int driveId = JsonUtil.getInt("/Modules/" + moduleName + "/driveMotorId", JSON);
    int steerId = JsonUtil.getInt("/Modules/" + moduleName + "/steerMotorId", JSON);
    int encId = JsonUtil.getInt("/Modules/" + moduleName + "/cancoderId", JSON);
    Angle offset = Rotations.of(radToRot(JsonUtil.getDouble("/Modules/" + moduleName + "/offsetRadians", JSON)));
    boolean steerInv = new JsonUtil().getBool("/Modules/" + moduleName + "/steerMotorInverted", JSON);
    boolean encInv = new JsonUtil().getBool("/Modules/" + moduleName + "/encoderInverted", JSON);
    Distance x = Inches.of(JsonUtil.getDouble("/Modules/" + moduleName + "/Position/xPosition", JSON));
    Distance y = Inches.of(JsonUtil.getDouble("/Modules/" + moduleName + "/Position/yPosition", JSON));

    return ConstantCreator.createModuleConstants(
      steerId, driveId, encId, offset, x, y, invertSide, steerInv, encInv);
  }

  private static double radToRot(double radians) {
    return radians / (2.0 * Math.PI);
  }

  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(SwerveDrivetrainConstants dc,
                   SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, dc, modules);
    }

    public TunerSwerveDrivetrain(SwerveDrivetrainConstants dc, double odoHz,
                   SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, dc, odoHz, modules);
    }

    public TunerSwerveDrivetrain(SwerveDrivetrainConstants dc, double odoHz,
                   Matrix<N3, N1> odoStd, Matrix<N3, N1> visStd,
                   SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, dc, odoHz, odoStd, visStd, modules);
    }
  }
}
