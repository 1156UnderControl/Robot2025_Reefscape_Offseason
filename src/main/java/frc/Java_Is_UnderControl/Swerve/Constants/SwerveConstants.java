package frc.Java_Is_UnderControl.Swerve.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

public class SwerveConstants {
  public static final String FRONT_LEFT_MODULE_NAME = "Front Left Module";
  public static final String FRONT_RIGHT_MODULE_NAME = "Front Right Module";
  public static final String BACK_LEFT_MODULE_NAME = "Back Left Module";
  public static final String BACK_RIGHT_MODULE_NAME = "Back Right Module";

  public static final int MAX_NUMBER_CONFIG_ATTEMPTS = 5;
  public static final double TIMEOUT_SECONDS_CONFIG = 0.25;

  public static final double HIGH_FREQUENCY = 250;
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double LOW_FREQUENCY = 50;

  public static final Lock odometryUpdatesWhileReadingDataStopper = new ReentrantLock();

  public static final double IS_CONNECTED_DEBOUNCE_TIME = 0.5;

  public static final double WHEEL_RADIUS_METERS = 0.04937;
  public static final double ROBOT_MASS = 64.3;
  public static final double ROBOT_MOI = 6.722;
  public static final double WHEEL_COF = 1.430;
  public static final double GEARBOX_REDUCTION = 1.430;
  public static final double ROBOT_SIZE = 0.830;

  public static final Translation2d[] MODULE_OFFSETS = {
    new Translation2d(10.375, 10.375),
    new Translation2d(10.375, -10.375),
    new Translation2d(-9.75, 9.75),
    new Translation2d(-10.375, -10.375)
  };

private static final Slot0Configs ksteerGains = new Slot0Configs()
    .withKP(80).withKI(0).withKD(0)
    .withKS(0.26).withKV(0).withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

private static final Slot0Configs kdriveGains = new Slot0Configs()
    .withKP(0.3).withKI(0).withKD(0)
    .withKS(0.074247).withKV(0.12407);

private static final Slot0Configs ksteerGainsBackLeft = new Slot0Configs()
    .withKP(50).withKI(0).withKD(0)
    .withKS(0.26).withKV(0).withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  private static final Current kSlipCurrent = Amps.of(120);

  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(Amps.of(60))
              .withStatorCurrentLimitEnable(true)
      );
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  private static final Pigeon2Configuration pigeonConfigs = null;

  public static final CANBus kCANBus = CANBus.systemCore(3);

  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.07);

  private static final double kCoupleRatio = 3.5714285714285716;
  private static final double kDriveGearRatio = 6.746031746031747;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final Distance kWheelRadius = Inches.of(1.943852);
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;
  private static final int kPigeonId = 0;

  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);

  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  private static final double kSteerGearRatioBackLeft = 12.8;

  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs); 

    private static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getConstantCreator(double steerRatio, double driveRatio, Slot0Configs steerGains, Slot0Configs driveGains) {
        return new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(driveRatio)
            .withSteerMotorGearRatio(steerRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage); 
    }

    // Front Left
    private static final int kFrontLeftDriveMotorId = 5;
    private static final int kFrontLeftSteerMotorId = 6;
    private static final int kFrontLeftEncoderId = 1;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.046142578125);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false; 
    private static final Distance kFrontLeftXPos = Inches.of(10.375);
    private static final Distance kFrontLeftYPos = Inches.of(10.375); 
    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 2;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.42626953125);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;  
    private static final Distance kFrontRightXPos = Inches.of(10.375);
    private static final Distance kFrontRightYPos = Inches.of(-10.375); 
    // Back Left
    private static final int kBackLeftDriveMotorId = 9;
    private static final int kBackLeftSteerMotorId = 10;
    private static final int kBackLeftEncoderId = 3;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.02099609375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;  
    private static final Distance kBackLeftXPos = Inches.of(-9.75);
    private static final Distance kBackLeftYPos = Inches.of(9.75);  
    // Back Right
    private static final int kBackRightDriveMotorId = 11;
    private static final int kBackRightSteerMotorId = 12;
    private static final int kBackRightEncoderId = 4;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.473876953125);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false; 
    private static final Distance kBackRightXPos = Inches.of(-10.375);
    private static final Distance kBackRightYPos = Inches.of(-10.375);  

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        getConstantCreator(kSteerGearRatio, kDriveGearRatio, ksteerGains, kdriveGains).createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        getConstantCreator(kSteerGearRatio, kDriveGearRatio, ksteerGains, kdriveGains).createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        getConstantCreator(kSteerGearRatioBackLeft, kDriveGearRatio, ksteerGainsBackLeft, kdriveGains).createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        getConstantCreator(kSteerGearRatio, kDriveGearRatio, ksteerGains, kdriveGains).createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );  

    public static SwerveModuleConstants[] getModuleConstants() {
      SwerveModuleConstants[] arrayModules = {FrontLeft, FrontRight, BackLeft, BackRight};
      return arrayModules;
    }
        
    public static SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
      return DrivetrainConstants;
    } 

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        } 

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        } 

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
