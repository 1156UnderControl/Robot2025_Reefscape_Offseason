package frc.Java_Is_UnderControl.Motors;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Swerve.IO.Module.ModuleIO.ModuleIOInputs;

public interface MotorIO {

    final int maximumRetries = 5;

    @AutoLog
    public static class MotorIOInputs{
        public double appliedOutput = 0.0;
        public double targetOutput = 0.0;
        public double current = 0.0;
        public double position = 0.0;
        public double velocity = 0.0;
        public double temperature = 0.0;
        public int faults = 0;
        public double targetPosition = 0.0;
        public double targetSpeed = 0.0;
        public String description = "";
        public boolean isInverted = false;
    }

    void updateInputs(ModuleIOInputs inputs);

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

    void configureMotionProfiling(
        double P,
        double I,
        double D,
        double ff,
        double maxVelocity,
        double maxAcceleration,
        double positionErrorAllowed);

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

    void setTwoSysIDMotors(Subsystem currentSubsystem, MotorIO otherMotor);

    Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    Command sysIdDynamic(SysIdRoutine.Direction direction);

    void updateLogs();
}   
