package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
    OperatorController operatorController;
    ScorerSubsystem scorer;

public ScoreObjectPosition (ScorerSubsystem scorer){ 
    this.scorer = scorer; 


    addCommands(
        new MoveScorerToPrepareScore(scorer)
            .until(() -> (this.scorer.isElevatorAtTargetPosition() && this.scorer.isPivotAtTargetPosition()) && operatorController.scoreObject().getAsBoolean())
                .andThen(new MoveScorerToScorePosition(scorer))
                .until(operatorController.goToReefA())
                );
        
    }
}