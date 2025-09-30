package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranchFastAutonomousWithoutBackup extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBranchFastAutonomousWithoutBackup(SwerveSubsystem swerve, TargetBranch branch, boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.goDirect = goDirect;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (this.goDirect) {
      this.swerve.driveToBranch(targetBranch, false, true);
    } else {
      this.swerve.driveToBranch(targetBranch, false, false);
    }
    isGoingToNonBackupPosition = true;
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
