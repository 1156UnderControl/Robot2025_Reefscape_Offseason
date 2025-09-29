package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScoreCoralPosition;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
    DriverController driverController;
    OperatorController operatorController;
    ScorerSubsystem scorer;

    public ScoreObjectPosition (ScorerSubsystem scorer){ 
        this.scorer = scorer;
        this.driverController = DriverController.getInstance();
        this.operatorController = OperatorController.getInstance();
        addCommands(
            new MoveScorerToPrepareScore(scorer).until(operatorController.scoreObject()), 
            new MoveScorerToScoreCoralPosition(scorer).until(operatorController.cancelAction())
        );
    }
}