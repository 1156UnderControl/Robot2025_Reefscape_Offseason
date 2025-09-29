package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.scorer.ScorerIO;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToFace extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  ScorerIO superStructure;

  public SwerveGoToFace(SwerveSubsystem swerve, ScorerIO superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.superStructure = superStructure;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
  }

  @Override
  public void execute() {
    this.swerve.goToFaceTeleoperated(targetBranch);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetFacePositionWithoutHeading();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.stopSwerve();
  }
}
