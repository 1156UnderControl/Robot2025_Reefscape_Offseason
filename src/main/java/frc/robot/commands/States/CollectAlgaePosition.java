package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToCollectAlgae;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectAlgaePosition extends SequentialCommandGroup {

  public CollectAlgaePosition(ScorerSubsystem scorer, OperatorController keyboard) {
    addCommands(
      new MoveScorerToCollectAlgae(scorer),
      Commands.idle(scorer)
    );
  }
}
