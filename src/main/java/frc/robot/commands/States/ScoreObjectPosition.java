package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScoreCoral;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreObjectPosition extends SequentialCommandGroup {
    OperatorController operatorController = OperatorController.getInstance();
    ScorerSubsystem scorer = ScorerSubsystem.getInstance();

public ScoreObjectPosition (ScorerSubsystem scorer){ 

    addCommands(
        new Movescorertoprep()
            .until(() -> this.scorer.isElevatorAtTargetPosition() && this.scorer.isPivotAtTargetPosition()),
                Commands.waitUntil(operatorController.scoreObject())
                .andThen(new MoveScorerToScoreCoral())
                .until(operatorController.goToDefaultPosition())
                );
        
            }
        }
