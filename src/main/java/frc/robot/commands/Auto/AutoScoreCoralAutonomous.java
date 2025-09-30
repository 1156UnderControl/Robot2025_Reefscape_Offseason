package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScoreCoralPosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomous extends SequentialCommandGroup {

  public AutoScoreCoralAutonomous(IntakeSubsystem intake, ScorerSubsystem scorer, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(
        new CollectCoralAutonomous(intake, scorer).andThen(new MoveScorerToPrepareScore(scorer)).deadlineFor(new SwerveGoToBackupDirectAutonomous(swerve, branch, true)),
        new SwerveGoToBranchFastAutonomousWithoutBackup(swerve, branch, true),
        new MoveScorerToScoreCoralPosition(scorer).until(() -> !scorer.hasCoral()),
        new WaitCommand(0.3).alongWith(Commands.idle(scorer)),
        new SwerveGoToBackupDirectAutonomous(swerve, branch, true).alongWith(Commands.idle(scorer))
        );
  }
}
