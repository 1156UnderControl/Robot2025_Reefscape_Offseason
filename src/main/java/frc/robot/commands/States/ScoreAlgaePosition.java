package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class ScoreAlgaePosition extends SequentialCommandGroup {
  private ScorerSubsystem scorer;
  private OperatorController keyboard;
  
  public ScoreAlgaePosition(ScorerSubsystem scorer) {
    this.scorer = scorer;
    this.keyboard = keyboard;

    addCommands(
    new ScoreAlgaePosition(scorer)
    );
  }
}
