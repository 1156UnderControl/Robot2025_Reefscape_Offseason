package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.commands.Swerve.SwerveGoToBackupIfNecessary;
import frc.robot.commands.Swerve.SwerveGoToBranch;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class GoAndRaiseElevator extends SequentialCommandGroup {
  public GoAndRaiseElevator(SwerveSubsystem swerve, ScorerSubsystem scorer, TargetBranch branch) {
    addCommands(
      new SwerveGoToBackupIfNecessary(swerve, branch),
      new MoveScorerToPrepareScore(scorer)
        .alongWith(new SwerveGoToBranch(swerve, branch)));
  }
}
