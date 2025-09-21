package frc.robot.subsystems.swerve;

import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ISwerve {
  void driveLockedAngleToNearestCoralStation();

  void driveLockedAngleToClimb();

  void driveToNearestCoralStation();

  void driveToBranch(TargetBranch branch, boolean backup, boolean goDirect);

  void goToFaceTeleoperated(TargetBranch branch);

  void goToFaceAutonomous(TargetBranch branch);

  void goToCollectAlgaeFromFacePosition(TargetBranch branch);

  boolean isAtTargetFacePositionWithoutHeading();

  boolean checkBackupNecessary();

  boolean checkPivotWillCrashOnReef();

  void driveAlignAngleJoystick();

  void driveAlignAngleJoystickSuperSlow();

  void stopSwerve();

  double getDistanceToTargetBranch();

  boolean swerveIsToCloseToReefForLiftingElevador();

  boolean isAtTargetPositionWithHeading();

  boolean isAtTargetPositionWithoutHeading();

  void setTargetBranch(TargetBranch branch);
}
