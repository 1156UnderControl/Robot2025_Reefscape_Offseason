package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean isBackupNecessary;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBranch(SwerveSubsystem swerve, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
  }

  @Override
  public void execute() {
      this.swerve.driveToBranch(targetBranch, false, false);
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
