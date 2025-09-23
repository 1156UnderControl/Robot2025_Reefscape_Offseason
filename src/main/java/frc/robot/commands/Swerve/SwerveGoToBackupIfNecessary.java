package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBackupIfNecessary extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isBackupNecessary;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBackupIfNecessary(SwerveSubsystem swerve, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
    isBackupNecessary = this.swerve.checkBackupNecessary();
  }

  @Override
  public void execute() {
     this.swerve.driveToBranch(targetBranch, true, true);
      if (this.swerve.isAtTargetPositionWithoutHeading()) {
        reachedBackupPosition = true;
      }
  }

  @Override
  public boolean isFinished() {
    if(!isBackupNecessary){
      return true;
    }
    return reachedBackupPosition;
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
