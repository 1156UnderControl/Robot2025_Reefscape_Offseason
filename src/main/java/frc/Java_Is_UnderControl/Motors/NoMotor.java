package frc.Java_Is_UnderControl.Motors;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class NoMotor implements MotorIO {

  @Override
  public String getMotorName() {
    return "";
  }

  @Override
  public void factoryDefault() {
    return;
  }

  @Override
  public void clearStickyFaults() {
    return;
  }

  @Override
  public void configureFeedForward(double Kg, double Ks, double Kv) {
    return;
  }

  @Override
  public void setMaxMotorOutput(double maxOutput) {}

  @Override
  public void setMinMotorOutput(double minOutput) {}

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    return;
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    return;
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    return;
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    return;
  }

  @Override
  public void setInverted(boolean inverted) {
    return;
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    return;
  }

  @Override
  public void burnFlash() {
    return;
  }

  @Override
  public void set(double percentOutput) {
    return;
  }

  @Override
  public void set(Voltage percentOutput) {
    return;
  }

  @Override
  public void setPositionReference(double position) {
    return;
  }

  @Override
  public void setPositionReference(double position, double ArbFF) {
    return;
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
    return;
  }

  @Override
  public void configureMotionProfiling(
      double P,
      double I,
      double D,
      double kS,
      double kV,
      double kA,
      double maxVelocity,
      double maxAcceleration,
      double jerk) {
    return;
  }

  public void setVelocityReference(double velocity, double feedforward) {
    return;
  }

  public void setPositionReferenceMotionProfiling(double position, double arbFF) {
    return;
  }

  @Override
  public void configureTrapezoid(double maxAcceleration, double maxVelocity) {
    return;
  }

  @Override
  public void setPositionReferenceTrapezoid(
      double kDt, double positionGoal, double velocityGoal, double arbFF) {
    return;
  }

  @Override
  public void setPositionReferenceTrapezoid(double kDt, double positionGoal, double velocityGoal) {
    return;
  }

  @Override
  public double getVoltage() {
    return 0.0;
  }

  @Override
  public double getDutyCycleSetpoint() {
    return 0.0;
  }

  @Override
  public void setVoltage(double voltage) {
    return;
  }

  @Override
  public double getAppliedOutput() {
    return 0.0;
  }

  @Override
  public double getPosition() {
    return 0.0;
  }

  @Override
  public double getVelocity() {
    return 0.0;
  }

  @Override
  public void setPositionFactor(double factor) {
    return;
  }

  @Override
  public void setVelocityFactor(double factor) {
    return;
  }

  @Override
  public void setPosition(double position) {
    return;
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    return;
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    return;
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    return;
  }

  @Override
  public void setFollower(int leaderIDcan, boolean invert) {
    return;
  }

  @Override
  public Object getMotor() {
    return null;
  }

  @Override
  public void configureSysID(
      double quasistaticVoltagePerSecond, double dynamicVoltage, double timeoutSysID) {
    return;
  }

  @Override
  public void setSysID(Subsystem currentSubsystem) {
    return;
  }

  @Override
  public void setTwoSysIDMotors(Subsystem currentSubsystem, MotorIO otherMotor) {
    return;
  }

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.none();
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.none();
  }

  @Override
  public double getPositionExternalEncoder() {
    return 0.0;
  }

  @Override
  public double getPositionExternalAbsoluteEncoder() {
    return 0;
  }

  @Override
  public void setPositionFactorExternalEncoder(double factor) {
    return;
  }

  @Override
  public void setVelocityFactorExternalEncoder(double factor) {
    return;
  }

  @Override
  public void setPositionExternalEncoder(double position) {
    return;
  }

  @Override
  public void configExternalEncoder() {
    return;
  }

  @Override
  public double getVelocityExternalEncoder() {
    return 0.0;
  }

  @Override
  public void updateLogs() {
    return;
  }

  @Override
  public void setAbsoluteEncoderZeroOffset(double zeroOffset) {
    return;
  }
}
