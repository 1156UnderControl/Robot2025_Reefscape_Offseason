package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.commands.Scorer.MoveScorerToDefaultPosition;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectCoralPosition extends SequentialCommandGroup {

  private IntakeSubsystem intake;
  private ScorerSubsystem scorer;

  public CollectCoralPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {
    this.intake = intake;
    this.scorer = scorer;

    addCommands(
      new MoveScorerToDefaultPosition(scorer),
      new MoveIntakeToCollectPosition(intake)
        .onlyIf(() -> !this.scorer.hasCoral())
        .until(() -> this.intake.indexerHasCoral())
        .andThen(new MoveScorerToDefaultPosition(scorer))
        .andThen(new CollectCoralFromIndexer(scorer))
);
  }
}