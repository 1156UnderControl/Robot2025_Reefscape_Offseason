package frc.robot.commands.States;

import javax.naming.OperationNotSupportedException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.MoveScorerToCollectAlgae;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectAlgaePosition extends SequentialCommandGroup {

  private ScorerSubsystem scorer;
  private OperatorController keyboard;

  public CollectAlgaePosition(ScorerSubsystem scorer, OperatorController keyboard) {
  this.keyboard = keyboard;
  this.scorer = scorer;

    addCommands(
      new MoveScorerToCollectAlgae(scorer)
      .until(() -> keyboard.cancelAction().getAsBoolean() && this.scorer.isElevatorAtTargetPosition() && this.scorer.isPivotAtTargetPosition())
    );
  }
}
