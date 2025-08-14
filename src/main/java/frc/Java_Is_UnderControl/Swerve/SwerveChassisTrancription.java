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
            new Translation2d[] {new Translation2d(SwerveConstants.FrontLeft.LocationX, SwerveConstants.FrontLeft.LocationY),
                new Translation2d(SwerveConstants.FrontRight.LocationX, SwerveConstants.FrontRight.LocationY),
                new Translation2d(SwerveConstants.BackLeft.LocationX, SwerveConstants.BackLeft.LocationY),
                new Translation2d(SwerveConstants.BackRight.LocationX, SwerveConstants.BackRight.LocationY)});
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
