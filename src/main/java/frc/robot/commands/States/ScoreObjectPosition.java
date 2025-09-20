package frc.robot.commands.States;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
    DriverController driverController;
    OperatorController operatorController;
    ScorerSubsystem scorer;

public 
ScoreObjectPosition (ScorerSubsystem scorer){ 
    this.scorer = scorer; 
    this.driverController = DriverController.getInstance();
    addCommands(
        new MoveScorerToPrepareScore(scorer),
                Commands.waitUntil(driverController.a()),
                new MoveScorerToScorePosition(scorer)
                .until((driverController.b()))
                );
        
    }
}