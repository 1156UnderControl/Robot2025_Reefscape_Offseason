package frc.Java_Is_UnderControl.Motors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;

public class SparkMAXMotor implements MotorIO {

  private boolean factoryDefaultOcurred = false;

  private double maxVelocity = 0;

  private double maxAcceleration = 0;

  // I don't how it works, and if it's truly necessary to use
  private Supplier<Double> velocity;
  private Supplier<Double> position;

  private SparkMaxConfig config;
  private String motorName;
  private boolean isInverted = false;
  private boolean usingAlternateEncoder = false;


  private double targetPercentage;
  private double targetPosition;
  private double targetVelocity;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  public Velocity<VoltageUnit> quasistaticVoltagePerSecond = null;
  public Voltage dynamicVoltage = null;
  public Time timeoutSysID = null;

  private SysIdRoutine sysIdRoutine;
  private SparkMax motor;

  private TrapezoidProfile motionProfile;

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State trapezoidGoal = new TrapezoidProfile.State();

  private IdleMode lastTargetIdleMode = null;

  public SparkMAXMotor(int motorID, int busID, String motorName) {
    this(motorID, busID, false, motorName);
  }

  public SparkMAXMotor(int motorID, int busID, boolean usingAlternateEncoder, String motorName) {
    this.motor = new SparkMax(busID, motorID, MotorType.kBrushless);
    this.config = new SparkMaxConfig();
    this.motorName = motorName;
    this.usingAlternateEncoder = usingAlternateEncoder;
    clearStickyFaults();
    this.setAlternateEncoder(usingAlternateEncoder);
  }

  private void setAlternateEncoder(boolean usingAlternateEncoder) {
    if (usingAlternateEncoder) {
      this.config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      this.config.alternateEncoder.countsPerRevolution(8192);
    } else {
      this.config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
  }

  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public String getMotorName() {
    return this.motorName;
  }

  // To restore factory default its necessary to wait until revlib 2025 is
  // officially launched
  @Override
  public void factoryDefault() {
    if (!factoryDefaultOcurred) {}
    factoryDefaultOcurred = true;
  }

  @Override
  public void clearStickyFaults() {
    configureSparkMax(motor::clearFaults);
  }

  public void updateConfig(SparkMaxConfig cfgGiven) {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException(
          "Configuration changes cannot be applied while the robot is enabled.");
    }
    config.apply(cfgGiven);
    configureSparkMax(
        () ->
            motor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs){
    inputs.appliedOutput = this.getAppliedOutput();
    inputs.current = this.getVoltage();
    inputs.position = this.getPosition();
    inputs.velocity = this.getVelocity();
    inputs.temperature = this.motor.getMotorTemperature();
    inputs.faults = this.motor.getFaults().rawBits;
    inputs.targetPosition = this.targetPosition;
    inputs.targetSpeed = this.targetVelocity;
    inputs.isInverted = this.isInverted;
  }

  // To implement the Kg, Ks and Kv its necessary to wait until revlib 2025 is
  // officially launched
  @Override
  public void configureFeedForward(double Kg, double Ks, double Kv) {
    config.closedLoop.velocityFF(Kg);
  }

  @Override
  public void setMaxMotorOutput(double maxOutput) {}

  @Override
  public void setMinMotorOutput(double minOutput) {}

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    this.configurePIDF(P, I, D, F);
    config.closedLoop.iZone(Izone);
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    config.closedLoop.pidf(P, I, D, F);
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    config.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(minInput, maxInput);
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    IdleMode targetIdleMode = isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast;
    if (lastTargetIdleMode == targetIdleMode) {
      return;
    }
    config.idleMode(targetIdleMode);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    this.lastTargetIdleMode = targetIdleMode;
  }

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    config.alternateEncoder.inverted(inverted);
  }

  @Override
  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSparkMax(
        () -> {
          return motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
  }

  @Override
  public void set(double percentOutput) {
    if (percentOutput != this.targetPercentage) {
      this.motor
          .getClosedLoopController()
          .setReference(percentOutput, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0);
    }
    this.targetPercentage = percentOutput;
    this.targetPosition = Double.NaN;
    this.targetVelocity = Double.NaN;
  }

  @Override
  public void set(Voltage percentOutput) {
    if (percentOutput.in(Volts) != this.targetPercentage) {
      this.motor.set(percentOutput.in(Volts));
    }
    this.targetPercentage = percentOutput.in(Volts);
    this.targetPosition = Double.NaN;
    this.targetVelocity = Double.NaN;
  }

  @Override
  public void setPositionReference(double position) {
    if (this.getPosition() != position) {
      motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.targetPosition = position;
  }

  public void setPositionReference(double position, double arbFF) {
    if (this.getPosition() != position) {
      motor
          .getClosedLoopController()
          .setReference(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = Double.NaN;
    this.targetPosition = position;
  }

  @Override
  public void configureMotionProfiling(
      double P,
      double I,
      double D,
      double ff,
      double maxVelocity,
      double maxAcceleration,
      double positionErrorAllowed) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    config
        .closedLoop
        .maxMotion
        .maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
        .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(positionErrorAllowed, ClosedLoopSlot.kSlot0);
    config
        .closedLoop
        .p(P, ClosedLoopSlot.kSlot0)
        .i(I, ClosedLoopSlot.kSlot0)
        .d(D, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void configureMotionProfiling(
      double P,
      double I,
      double D,
      double kS,
      double kV,
      double kA,
      double cruiseVelocity,
      double acceleration,
      double jerk) {
    configureMotionProfiling(P, I, D, 0, maxVelocity, maxAcceleration, 0.05);
  }

  @Override
  public void setPositionReferenceMotionProfiling(double position, double arbFF) {
    motor
        .getClosedLoopController()
        .setReference(
            position,
            SparkBase.ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            arbFF);
  }

  @Override
  public void configureTrapezoid(double maxAcceleration, double maxVelocity) {
    this.motionProfile = new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void setPositionReferenceTrapezoid(
      double kDt, double positionGoal, double velocityGoal, double arbFF) {
    this.trapezoidGoal = new TrapezoidProfile.State(positionGoal, velocityGoal);
    this.currentSetpoint = motionProfile.calculate(kDt, this.currentSetpoint, this.trapezoidGoal);
    setPositionReference(this.currentSetpoint.position, arbFF);
  }

  @Override
  public void setPositionReferenceTrapezoid(double kDt, double positionGoal, double velocityGoal) {
    this.trapezoidGoal = new TrapezoidProfile.State(positionGoal, velocityGoal);
    this.currentSetpoint = motionProfile.calculate(kDt, this.currentSetpoint, this.trapezoidGoal);
    setPositionReference(this.currentSetpoint.position);
  }

  public void setVelocityReference(double velocity, double arbFF) {
    if (this.getVelocity() != velocity) {
      motor
          .getClosedLoopController()
          .setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFF);
    }
    this.targetPercentage = Double.NaN;
    this.targetVelocity = velocity;
    this.targetPosition = Double.NaN;
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public double getDutyCycleSetpoint() {
    return motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public double getPosition() {
    if (usingAlternateEncoder) {
      return motor.getAlternateEncoder().getPosition();
    }
    return motor.getEncoder().getPosition();
  }

  @Override
  public double getVelocity() {
    if (usingAlternateEncoder) {
      return motor.getAlternateEncoder().getVelocity();
    }
    return motor.getEncoder().getVelocity();
  }

  @Override
  public void setPositionFactor(double factor) {
    if (usingAlternateEncoder) {
      config.alternateEncoder.positionConversionFactor(factor);
    } else {
      config.encoder.positionConversionFactor(factor);
    }
  }

  @Override
  public void setVelocityFactor(double factor) {
    if (usingAlternateEncoder) {
      config.alternateEncoder.velocityConversionFactor(factor);
    } else {
      config.encoder.velocityConversionFactor(factor);
    }
  }

  @Override
  public void setPosition(double position) {
    this.motor.getEncoder().setPosition(position);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    config.voltageCompensation(nominalVoltage);
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    config.smartCurrentLimit(currentLimit);
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    config.closedLoopRampRate(rampRate).openLoopRampRate(rampRate);
  }

  @Override
  public void setFollower(int leaderIDcan, boolean invert) {
    config.follow(leaderIDcan, invert);
  }

  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public void configureSysID(
      double quasistaticVoltagePerSecond, double dynamicVoltage, double timeoutSysID) {
    this.quasistaticVoltagePerSecond = Volts.of(quasistaticVoltagePerSecond).per(Second);
    this.dynamicVoltage = Volts.of(dynamicVoltage);
    this.timeoutSysID = Seconds.of(timeoutSysID);
  }

  @Override
  public void setSysID(Subsystem currentSubsystem) {
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                this.quasistaticVoltagePerSecond, this.dynamicVoltage, this.timeoutSysID),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  this.set(voltage);
                },
                log -> {
                  log.motor(this.motorName)
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              this.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                      .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                      .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));
                },
                currentSubsystem));
  }

  @Override
  public void setTwoSysIDMotors(Subsystem currentSubsystem, MotorIO otherMotor) {
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                this.quasistaticVoltagePerSecond, this.dynamicVoltage, this.timeoutSysID),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  this.set(voltage);
                  otherMotor.set(voltage);
                },
                log -> {
                  log.motor(this.motorName)
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              this.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                      .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                      .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));

                  log.motor(otherMotor.getMotorName())
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              otherMotor.getAppliedOutput() * RobotController.getBatteryVoltage(),
                              Volts))
                      .linearPosition(m_distance.mut_replace(otherMotor.getPosition(), Meters))
                      .linearVelocity(
                          m_velocity.mut_replace(otherMotor.getVelocity(), MetersPerSecond));
                },
                currentSubsystem));
  }

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.quasistatic(direction);
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.dynamic(direction);
  }

  @Override
  public double getPositionExternalEncoder() {
    return motor.getAlternateEncoder().getPosition();
  }

  @Override
  public double getPositionExternalAbsoluteEncoder() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void setPositionFactorExternalEncoder(double factor) {
    this.config.alternateEncoder.positionConversionFactor(factor);
  }

  @Override
  public void setPositionExternalEncoder(double position) {
    this.motor.getAlternateEncoder().setPosition(position);
  }

  @Override
  public void configExternalEncoder() {
    this.config.alternateEncoder.countsPerRevolution(8192);
  }

  @Override
  public double getVelocityExternalEncoder() {
    return motor.getAlternateEncoder().getVelocity();
  }

  @Override
  public void setVelocityFactorExternalEncoder(double factor) {
    this.config.alternateEncoder.velocityConversionFactor(factor);
  }

  @Override
  public void setAbsoluteEncoderZeroOffset(double zeroOffset) {
    this.config.absoluteEncoder.zeroOffset(zeroOffset);
  }
}
