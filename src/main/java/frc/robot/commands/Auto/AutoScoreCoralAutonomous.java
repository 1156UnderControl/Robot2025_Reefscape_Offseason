package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScoreCoralPosition;
import frc.robot.commands.States.DefaultPosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralAutonomous extends SequentialCommandGroup{
    public AutoScoreCoralAutonomous(ScorerSubsystem scorer, IntakeSubsystem intake, SwerveSubsystem swerve, TargetBranch branch){
        addCommands(
        new InstantCommand(() -> scorer.overrideHasCoral()),
        new SwerveGoToBackupDirectAutonomous(swerve, branch, true).alongWith(new MoveScorerToPrepareScore(scorer)).until(() -> scorer.isScorerAtTargetPosition() && swerve.isAtTargetPositionWithoutHeading()),
        new SwerveGoToBranchFastAutonomousWithoutBackup(swerve, branch, true),
        new MoveScorerToScoreCoralPosition(scorer).until(() -> !scorer.hasCoral()),
        new SwerveGoToBackupDirectAutonomous(swerve, branch, true).until(() -> swerve.isAtTargetPositionWithoutHeading()),
        new DefaultPosition(intake, scorer)
        );
  }
}
