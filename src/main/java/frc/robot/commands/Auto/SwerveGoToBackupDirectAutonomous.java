package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBackupDirectAutonomous extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  boolean goDirect;

  public SwerveGoToBackupDirectAutonomous(SwerveSubsystem swerve, TargetBranch branch, boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.goDirect = goDirect;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
  }

  @Override
  public void execute() {
    this.swerve.driveToBranch(targetBranch, true, this.goDirect);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading();
  }

  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      this.swerve.stopSwerve();
    }
  }
}
