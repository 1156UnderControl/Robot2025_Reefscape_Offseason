package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.commands.Intake.MoveIntakeToHomedPosition;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.commands.Scorer.MoveScorerToDefaultAlgaePosition;
import frc.robot.commands.Scorer.MoveScorerToDefaultCoralPosition;
import frc.robot.commands.Scorer.MoveScorerToDefaultPosition;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class DefaultPosition extends SequentialCommandGroup {

  public DefaultPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {
    addCommands(
      Commands.either(new MoveScorerToDefaultCoralPosition(scorer), 
                      Commands.either(new MoveScorerToDefaultAlgaePosition(scorer),
                                      new MoveScorerToDefaultPosition(scorer), 
                                      () -> scorer.hasAlgae()), 
                      () -> scorer.hasCoral()),
      new MoveIntakeToHomedPosition(intake),
      Commands.idle(intake, scorer));
  }
}