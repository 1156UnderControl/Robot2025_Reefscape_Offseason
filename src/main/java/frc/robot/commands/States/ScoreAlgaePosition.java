package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.ExpellAlgae;
import frc.robot.commands.Scorer.MoveScorerToScoreAlgaePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreAlgaePosition extends SequentialCommandGroup {

ScorerSubsystem scorer;
OperatorController keyboard;

  public ScoreAlgaePosition(ScorerSubsystem scorer, OperatorController keyboard) {
    this.scorer = scorer;
    this.keyboard = keyboard;

    addCommands(
      new MoveScorerToScoreAlgaePosition(scorer)
          .until(() -> keyboard.scoreObject().getAsBoolean()
              && this.scorer.isElevatorAtTargetPosition()
              && this.scorer.isPivotAtTargetPosition()),
  
      new ExpellAlgae(scorer)
          .until(() -> keyboard.cancelAction().getAsBoolean())
  );
  }
}
