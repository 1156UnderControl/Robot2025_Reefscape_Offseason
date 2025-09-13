package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;

import java.util.HashMap;
import java.util.Optional;

public class SwerveChassisTrancription {
  private final SwerveDriveKinematics kinematics;
  private final HashMap<String, Optional<SwerveModuleState>> moduleInputs;

  public SwerveChassisTrancription() {
    this.moduleInputs = new HashMap<>();
    this.kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                SwerveConstants.getModuleConstants()[0].LocationX,
                SwerveConstants.getModuleConstants()[0].LocationY),
            new Translation2d(
                SwerveConstants.getModuleConstants()[1].LocationX,
                SwerveConstants.getModuleConstants()[1].LocationY),
            new Translation2d(
                SwerveConstants.getModuleConstants()[2].LocationX,
                SwerveConstants.getModuleConstants()[2].LocationY),
            new Translation2d(
                SwerveConstants.getModuleConstants()[3].LocationX,
                SwerveConstants.getModuleConstants()[3].LocationY));
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    this.moduleInputs.put(
        SwerveConstants.FRONT_LEFT_MODULE_NAME, Optional.of(swerveModuleStates[0]));
    this.moduleInputs.put(
        SwerveConstants.FRONT_RIGHT_MODULE_NAME, Optional.of(swerveModuleStates[1]));
    this.moduleInputs.put(
        SwerveConstants.BACK_LEFT_MODULE_NAME, Optional.of(swerveModuleStates[2]));
    this.moduleInputs.put(
        SwerveConstants.BACK_RIGHT_MODULE_NAME, Optional.of(swerveModuleStates[3]));
  }

  public void setChassisSpeedsWithDifferentCenterOfRotation(
      ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
    this.moduleInputs.put(
        SwerveConstants.FRONT_LEFT_MODULE_NAME, Optional.of(swerveModuleStates[0]));
    this.moduleInputs.put(
        SwerveConstants.FRONT_RIGHT_MODULE_NAME, Optional.of(swerveModuleStates[1]));
    this.moduleInputs.put(
        SwerveConstants.BACK_LEFT_MODULE_NAME, Optional.of(swerveModuleStates[2]));
    this.moduleInputs.put(
        SwerveConstants.BACK_RIGHT_MODULE_NAME, Optional.of(swerveModuleStates[3]));
  }

  public HashMap<String, Optional<SwerveModuleState>> getModuleInputs() {
    return this.moduleInputs;
  }
}
