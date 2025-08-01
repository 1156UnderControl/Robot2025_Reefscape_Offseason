package frc.Java_Is_UnderControl.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.IO.Gyro.GyroIO;
import frc.Java_Is_UnderControl.Swerve.IO.Gyro.GyroIO.GyroIOInputs;
import frc.Java_Is_UnderControl.Swerve.IO.Gyro.GyroIOPigeon2;
import frc.Java_Is_UnderControl.Swerve.IO.Module.ModuleIO;
import frc.Java_Is_UnderControl.Swerve.IO.Module.ModuleIOTalonFX;
import frc.Java_Is_UnderControl.Swerve.IO.Module.ModuleIO.ModuleIOInputs;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BiFunction;

public class Drive {

  private HashMap<String, Optional<SwerveModuleState>> targetModuleStates;
  private HashMap<String, Optional<SwerveModuleState>> lastTargetModuleStates;

  private final ModuleIO frontLeftModule;
  private final ModuleIO frontRightModule;
  private final ModuleIO backLeftModule;
  private final ModuleIO backRightModule;
  private final GyroIO pigeon;

  private static final ModuleIOInputs frontLeftInputs = new ModuleIOInputs();
  private static final ModuleIOInputs frontRightInputs = new ModuleIOInputs();
  private static final ModuleIOInputs backLeftInputs = new ModuleIOInputs();
  private static final ModuleIOInputs backRightInputs = new ModuleIOInputs();

  private static final GyroIOInputs pigeonInputs = new GyroIOInputs();

  private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
      SwerveConstants.MODULE_OFFSETS
    );

  public Drive(
      HashMap<
              String,
              Optional<
                  SwerveModuleConstants<
                      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>>
          motorConstants,
      GyroIOPigeon2 pigeon) {
    this.frontLeftModule =
        new ModuleIOTalonFX(
            motorConstants
                .get(SwerveConstants.FRONT_LEFT_MODULE_NAME)
                .orElseThrow(
                    () ->
                        new Error(
                            "The module named "
                                + SwerveConstants.FRONT_LEFT_MODULE_NAME
                                + " does not have a SwerveModuleConstant defined",
                            new Throwable("Null motor and encoder configs"))));
    this.frontRightModule =
        new ModuleIOTalonFX(
            motorConstants
                .get(SwerveConstants.FRONT_RIGHT_MODULE_NAME)
                .orElseThrow(
                    () ->
                        new Error(
                            "The module named "
                                + SwerveConstants.FRONT_RIGHT_MODULE_NAME
                                + " does not have a SwerveModuleConstant defined",
                            new Throwable("Null motor encoder configs"))));
    this.backLeftModule =
        new ModuleIOTalonFX(
            motorConstants
                .get(SwerveConstants.BACK_LEFT_MODULE_NAME)
                .orElseThrow(
                    () ->
                        new Error(
                            "The module named "
                                + SwerveConstants.BACK_LEFT_MODULE_NAME
                                + " does not have a SwerveModuleConstant defined",
                            new Throwable("Null motor encoder configs"))));
    this.backRightModule =
        new ModuleIOTalonFX(
            motorConstants
                .get(SwerveConstants.BACK_RIGHT_MODULE_NAME)
                .orElseThrow(
                    () ->
                        new Error(
                            "The module named "
                                + SwerveConstants.BACK_RIGHT_MODULE_NAME
                                + " does not have a SwerveModuleConstant defined",
                            new Throwable("Null motor encoder configs"))));
    this.pigeon = pigeon;
  }

  public void updateModuleTargetStates(HashMap<String, Optional<SwerveModuleState>> moduleStates) {
    this.targetModuleStates = moduleStates;
    this.setInputs();
  }

  private SwerveModuleState getTargetModuleState(String moduleName) {
    BiFunction<String,Optional<SwerveModuleState>,SwerveModuleState> resolve = (name, optionalSwerveModuleState) -> {
      if (optionalSwerveModuleState != null && optionalSwerveModuleState.isPresent()) {
        this.lastTargetModuleStates.put(name, optionalSwerveModuleState);
        return optionalSwerveModuleState.get();
      }
      Optional<SwerveModuleState> cached = this.lastTargetModuleStates.get(name);
      if (cached != null && cached.isPresent()) {
        return cached.get();
      }
      throw new Error("Your " + name + " current module state is empty", new Throwable("Null motor encoder configs"));
      };

    return resolve.apply(moduleName, targetModuleStates.get(moduleName));
  }

  private Rotation2d getModuleSteerTargetAngle(String moduleName) {
    return this.getTargetModuleState(moduleName).angle;
  }

  private double getModuleDriveTargetSpeed(String moduleName) {
    return this.getTargetModuleState(moduleName).speed;
  }

  private void setInputs() {
    this.updateModuleInputs();
    this.updatePigeonInputs();
    this.setModuleInputs();
  }

  private void setModuleInputs() {
    this.frontLeftModule.setDriveVelocity(
        this.getModuleDriveTargetSpeed(SwerveConstants.FRONT_LEFT_MODULE_NAME));
    this.frontRightModule.setDriveVelocity(
        this.getModuleDriveTargetSpeed(SwerveConstants.FRONT_RIGHT_MODULE_NAME));
    this.backLeftModule.setDriveVelocity(
        this.getModuleDriveTargetSpeed(SwerveConstants.BACK_LEFT_MODULE_NAME));
    this.backRightModule.setDriveVelocity(
        this.getModuleDriveTargetSpeed(SwerveConstants.BACK_RIGHT_MODULE_NAME));

    this.frontLeftModule.setSteerPosition(
        this.getModuleSteerTargetAngle(SwerveConstants.FRONT_LEFT_MODULE_NAME));
    this.frontRightModule.setSteerPosition(
        this.getModuleSteerTargetAngle(SwerveConstants.FRONT_RIGHT_MODULE_NAME));
    this.backLeftModule.setSteerPosition(
        this.getModuleSteerTargetAngle(SwerveConstants.BACK_LEFT_MODULE_NAME));
    this.backRightModule.setSteerPosition(
        this.getModuleSteerTargetAngle(SwerveConstants.BACK_RIGHT_MODULE_NAME));
  }

  private void updateModuleInputs() {
    this.frontLeftModule.updateInputs(frontLeftInputs);
    this.frontRightModule.updateInputs(frontRightInputs);
    this.backLeftModule.updateInputs(backLeftInputs);
    this.backRightModule.updateInputs(backRightInputs);
  }

  private void updatePigeonInputs() {
    this.pigeon.updateInputs(pigeonInputs);
  }

  public ChassisSpeeds getRobotSpeeds(){
    return kinematics.toChassisSpeeds(this.frontLeftModule.getCurrentModuleState(), this.frontRightModule.getCurrentModuleState(), 
    this.backLeftModule.getCurrentModuleState(), this.backRightModule.getCurrentModuleState());
  }

  public Rotation2d getRobotAngle(){
    return this.pigeon.getInputs().yawPosition;
  }
}
