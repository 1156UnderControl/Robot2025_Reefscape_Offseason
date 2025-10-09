
package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class MoveScorerToScoreAlgaePosition extends Command {

private ScorerSubsystem scorer;

  public MoveScorerToScoreAlgaePosition(ScorerSubsystem scorer) {
    this.scorer = scorer;
    addRequirements(scorer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.scorer.moveScorerToScoreAlgae();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
