package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectCoralPosition extends SequentialCommandGroup {

  ScorerSubsystem scorer;
  IntakeSubsystem intake;


  public CollectCoralPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {
    this.scorer = scorer;
    this.intake = intake;

    addCommands(
      new MoveIntakeToCollectPosition(intake),
      new CollectCoralFromIndexer(scorer, intake));
  }
}