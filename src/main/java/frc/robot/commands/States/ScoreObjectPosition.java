package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
    OperatorController operatorController = OperatorController.getInstance();
    ScorerSubsystem scorer = ScorerSubsystem.getInstance();

    public ScoreObjectPosition (ScorerSubsystem scorer){ 
        addCommands(
            new MoveScorerToPrepareScore(this.scorer),
            Commands.waitUntil(operatorController.scoreObject())
                .andThen(new MoveScorerToScorePosition(this.scorer))
        );
    }
}
