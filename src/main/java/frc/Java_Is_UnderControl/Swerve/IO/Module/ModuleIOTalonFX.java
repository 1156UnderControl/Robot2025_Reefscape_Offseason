package frc.Java_Is_UnderControl.Swerve.IO.Module;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.IO.PhoenixOdometryThread;
import frc.robot.subsystems.generated.TunerConstants;

import java.util.Queue;

public class ModuleIOTalonFX implements ModuleIO {

  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final CANcoder encoder;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> steerAbsolutePosition;
  private final StatusSignal<Angle> steerPosition;
  private final Queue<Double> steerPositionQueue;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerAppliedVolts;
  private final StatusSignal<Current> steerCurrent;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer steerConnectedDebounce = new Debouncer(0.5);
  private final Debouncer steerEncoderConnectedDebounce = new Debouncer(0.5);

  private ModuleIOInputs moduleInputs;

  private SwerveModuleState previousModuleState;

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    this.driveTalon =
        new TalonFX(this.constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    this.steerTalon =
        new TalonFX(this.constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    this.encoder =
        new CANcoder(this.constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

    this.configureDriveMotor(this.constants, this.driveTalon);
    this.configureSteerMotor(this.constants, this.steerTalon);
    this.configureEncoder(this.constants, this.encoder);

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    steerAbsolutePosition = encoder.getAbsolutePosition();
    steerPosition = steerTalon.getPosition();
    steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerPosition.clone());
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerCurrent = steerTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveConstants.HIGH_FREQUENCY, drivePosition, steerPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        SwerveConstants.LOW_FREQUENCY,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        steerAbsolutePosition,
        steerVelocity,
        steerAppliedVolts,
        steerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    this.previousModuleState = new SwerveModuleState();
    this.moduleInputs = new ModuleIOInputs();
  }

  private void configureDriveMotor(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      TalonFX driveTalon) {
    TalonFXConfiguration configuration = constants.DriveMotorInitialConfigs;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Slot0 = constants.DriveMotorGains;
    configuration.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    configuration.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    configuration.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    configuration.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(
        SwerveConstants.MAX_NUMBER_CONFIG_ATTEMPTS,
        () ->
            driveTalon
                .getConfigurator()
                .apply(configuration, SwerveConstants.TIMEOUT_SECONDS_CONFIG));
    tryUntilOk(
        SwerveConstants.MAX_NUMBER_CONFIG_ATTEMPTS,
        () -> driveTalon.setPosition(0.0, SwerveConstants.TIMEOUT_SECONDS_CONFIG));
  }

  private void configureSteerMotor(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      TalonFX steerTalon) {
    var configuration = new TalonFXConfiguration();
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configuration.Slot0 = constants.SteerMotorGains;
    configuration.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    configuration.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
        };
    configuration.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    configuration.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    configuration.MotionMagic.MotionMagicAcceleration =
        configuration.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    configuration.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    configuration.MotionMagic.MotionMagicExpo_kA = 0.1;
    configuration.ClosedLoopGeneral.ContinuousWrap = true;
    configuration.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(
        SwerveConstants.MAX_NUMBER_CONFIG_ATTEMPTS,
        () ->
            steerTalon
                .getConfigurator()
                .apply(configuration, SwerveConstants.TIMEOUT_SECONDS_CONFIG));
  }

  private void configureEncoder(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      CANcoder encoder) {
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(cancoderConfig);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    StatusCode driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    StatusCode steerStatus =
        BaseStatusSignal.refreshAll(steerPosition, steerVelocity, steerAppliedVolts, steerCurrent);
    StatusCode steerEncoderStatus = BaseStatusSignal.refreshAll(steerAbsolutePosition);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.steerConnected = steerConnectedDebounce.calculate(steerStatus.isOK());
    inputs.steerEncoderConnected =
        steerEncoderConnectedDebounce.calculate(steerEncoderStatus.isOK());
    inputs.steerAbsolutePosition =
        Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
    inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometrySteerPositions =
        steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();

    this.moduleInputs = inputs;
  }

  @Override
  public void setDriveVelocity(double metersPerSecond) {
    double velocityRotPerSec = metersPerSecond / (SwerveConstants.WHEEL_RADIUS_METERS * (2 * Math.PI));
    System.out.println("set velocity:" + velocityRotPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setSteerOpenLoop(double output) {
    steerTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public SwerveModuleState getCurrentModuleState(){
    if (!this.moduleInputs.driveConnected || !this.moduleInputs.steerEncoderConnected) {
        return this.previousModuleState;
    }
  
    double speedMetersPerSecond = this.moduleInputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS;
    Rotation2d angle = this.moduleInputs.steerPosition;

    SwerveModuleState state = new SwerveModuleState(speedMetersPerSecond, angle);
    previousModuleState = state;
    return state;
  }
}
