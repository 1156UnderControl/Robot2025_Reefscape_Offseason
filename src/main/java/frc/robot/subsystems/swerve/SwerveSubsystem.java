package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Swerve.BaseSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.Configs.BaseSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.Configs.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.robot.joysticks.DriverController;

import java.util.HashMap;

public class SwerveSubsystem extends SubsystemBase {
  private final BaseSwerveSubsystem swerve;
  private final HashMap<
          String,
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      moduleConstants;
  private final DriverController driverController;

  private double targetCOSRotation;
  private double targetSINRotation;

  public SwerveSubsystem() {
    this.moduleConstants = new HashMap<>();

    this.moduleConstants.put(SwerveConstants.FRONT_LEFT_MODULE_NAME, SwerveConstants.FrontLeft);

    this.moduleConstants.put(SwerveConstants.FRONT_RIGHT_MODULE_NAME, SwerveConstants.FrontRight);

    this.moduleConstants.put(SwerveConstants.BACK_LEFT_MODULE_NAME, SwerveConstants.BackLeft);

    this.moduleConstants.put(SwerveConstants.BACK_RIGHT_MODULE_NAME, SwerveConstants.BackRight);

    this.driverController = DriverController.getInstance();

    this.swerve =
        new BaseSwerveSubsystem(
            new BaseSwerveConfig(
                1,
                new SwervePathPlannerConfig(
                    new PIDConstants(5, 0.1, 0.1),
                    new PIDConstants(5, 0.1, 0.1),
                    new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720))),
                new PIDConfig(0.5, 0, 0)),
            this.moduleConstants);

    this.targetCOSRotation = 0;
    this.targetSINRotation = 0;
  }

  public void driveFieldOrientedLockedJoystickAngle() {
    ChassisSpeeds joystickSpeeds =
        swerve.inputsToChassisSpeeds(
            driverController.getXtranslation(), driverController.getYtranslation());
    this.swerve.driveFieldOrientedLockedAngle(
        joystickSpeeds, this.getJoystickCOSTargetRotation(), this.getJoystickSINTargetRotation());
  }

  private double getJoystickCOSTargetRotation(){
    if((this.driverController.getCOS_Joystick() != this.applyJoystickDeadBand(this.driverController.getCOS_Joystick()))){
      return this.targetCOSRotation;
    } else {
      this.targetCOSRotation = this.driverController.getCOS_Joystick();
      return this.targetCOSRotation;
    }
  }

  private double getJoystickSINTargetRotation(){
    if((this.driverController.getSIN_Joystick() != this.applyJoystickDeadBand(this.driverController.getSIN_Joystick()))){
      return this.targetSINRotation;
    } else {
      this.targetSINRotation = this.driverController.getSIN_Joystick();
      return this.targetSINRotation;
    }
  }

  private double applyJoystickDeadBand(double joystickValue){
    if(joystickValue < 0.25 && joystickValue > -0.25){
      return 0;
    } else {
      return joystickValue;
    }
  }

  @Override
  public void periodic() {
    this.swerve.periodic();
  }
}
