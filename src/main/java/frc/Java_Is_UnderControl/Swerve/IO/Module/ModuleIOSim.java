package frc.Java_Is_UnderControl.Swerve.IO.Module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using
 * a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // PID and model constants
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035; // (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double STEER_KP = 8.0;
  private static final double STEER_KD = 0.0;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor STEER_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;

  private boolean driveClosedLoop = false;
  private boolean steerClosedLoop = false;

  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController steerController = new PIDController(STEER_KP, 0, STEER_KD);

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {

    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
        DRIVE_GEARBOX);

    steerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            STEER_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
        STEER_GEARBOX);

    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }

    if (steerClosedLoop) {
      steerAppliedVolts = steerController.calculate(steerSim.getAngularPosition());
    } else {
      steerController.reset();
    }

    // Apply voltages and update sim
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    steerSim.setInputVoltage(MathUtil.clamp(steerAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    steerSim.update(0.02);

    // Drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    // Steer inputs
    inputs.steerConnected = true;
    inputs.steerEncoderConnected = true;
    inputs.steerAbsolutePosition = new Rotation2d(steerSim.getAngularPosition());
    inputs.steerPosition = new Rotation2d(steerSim.getAngularPosition());
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocity();
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDraw());

    // Odometry inputs
    inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    inputs.odometrySteerPositions = new Rotation2d[] { inputs.steerPosition };
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setSteerOpenLoop(double output) {
    steerClosedLoop = false;
    steerAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerClosedLoop = true;
    steerController.setSetpoint(rotation.getRadians());
  }

  @Override
  public SwerveModuleState getCurrentModuleState() {
    return new SwerveModuleState(
        driveSim.getAngularVelocity(), new Rotation2d(steerSim.getAngularPosition()));
  }
}
