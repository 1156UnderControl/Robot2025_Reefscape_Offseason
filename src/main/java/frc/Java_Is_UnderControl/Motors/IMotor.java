package frc.Java_Is_UnderControl.Motors;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface IMotor {
  final int maximumRetries = 5;

  String getMotorName();

  void factoryDefault();

  void clearStickyFaults();

  void configureFeedForward(double Kg, double Ks, double Kv);

  void setMaxMotorOutput(double maxOutput);

  void setMinMotorOutput(double minOutput);

  void configurePIDF(double P, double I, double D, double F, double Izone);

  void configurePIDF(double P, double I, double D, double F);

  void configurePIDWrapping(double minInput, double maxInput);

  void setMotorBrake(boolean isBrakeMode);

  void setInverted(boolean inverted);

  void setInvertedEncoder(boolean inverted);

  void burnFlash();

  void set(double percentOutput);

  void set(Voltage percentOutput);

  void setPositionReference(double position);

  void setPositionReference(double position, double arbFF);

  /**
   * Uses motion magic for TalonFXs and MAXMotion for SparkBases
   *
   * @param maxVelocity Max velocity that the motion profiling is allowed to use
   *     <ul>
   *       <li><b>Units:</b> rot per sec for TalonFXs, Spark will be the one used in the conversion
   *           factor (native is rotations)
   *     </ul>
   *
   * @param maxAcceleration Max acceleration that the motion profiling is allowed to use
   *     <ul>
   *       <li><b>Units:</b> rot per sec² for TalonFXs, Spark will be the one used in the conversion
   *           factor (native is rotations)
   *     </ul>
   */
  void configureMotionProfiling(
      double P,
      double I,
      double D,
      double ff,
      double maxVelocity,
      double maxAcceleration,
      double positionErrorAllowed);

  /**
   * Uses motion magic for TalonFXs and MAXMotion for SparkBases
   *
   * @param maxVelocity Max velocity that the motion profiling is allowed to use
   *     <ul>
   *       <li><b>Units:</b> rot per sec for TalonFXs, Spark will be the one used in the conversion
   *           factor (native is rotations)
   *     </ul>
   *     <p>
   * @param maxAcceleration Max acceleration that the motion profiling is allowed to use
   *     <ul>
   *       <li><b>Units:</b> rot per sec² for TalonFXs, Spark will be the one used in the conversion
   *           factor (native is rotations)
   *     </ul>
   *     <p>
   * @param jerk This is the target jerk (acceleration derivative) that the motion profiling is
   *     allowed to use - ONLY FOR TALONFX
   *     <ul>
   *       <li><b>Units:</b> rot per sec³
   *     </ul>
   */
  void configureMotionProfiling(
      double P,
      double I,
      double D,
      double kS,
      double kV,
      double kA,
      double maxVelocity,
      double maxAcceleration,
      double jerk);

  void setPositionReferenceMotionProfiling(double position, double arbFF);

  void configureTrapezoid(double maxAcceleration, double maxVelocity);

  void setPositionReferenceTrapezoid(double kDt, double positionGoal, double velocityGoal);

  void setPositionReferenceTrapezoid(
      double kDt, double positionGoal, double velocityGoal, double arbFF);

  double getVoltage();

  double getDutyCycleSetpoint();

  void setVoltage(double voltage);

  double getAppliedOutput();

  double getVelocity();

  double getPosition();

  double getPositionExternalEncoder();

  double getPositionExternalAbsoluteEncoder();

  double getVelocityExternalEncoder();

  void setPositionFactor(double factor);

  void setPositionFactorExternalEncoder(double factor);

  void setVelocityFactor(double factor);

  void setAbsoluteEncoderZeroOffset(double zeroOffset);

  void setVelocityFactorExternalEncoder(double factor);

  void setPosition(double position);

  void configExternalEncoder();

  void setPositionExternalEncoder(double position);

  void setVoltageCompensation(double nominalVoltage);

  void setCurrentLimit(int currentLimit);

  void setLoopRampRate(double rampRate);

  void setFollower(int leaderIDcan, boolean invert);

  Object getMotor();

  void configureSysID(
      double quasistaticVoltagePerSecond, double dynamicVoltage, double timeoutSysID);

  void setSysID(Subsystem currentSubsystem);

  void setTwoSysIDMotors(Subsystem currentSubsystem, IMotor otherMotor);

  Command sysIdQuasistatic(SysIdRoutine.Direction direction);

  Command sysIdDynamic(SysIdRoutine.Direction direction);

  void updateLogs();
}
