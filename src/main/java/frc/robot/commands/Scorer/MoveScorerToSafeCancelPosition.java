package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class MoveScorerToSafeCancelPosition extends Command {

ScorerSubsystem scorer;

  public MoveScorerToSafeCancelPosition(ScorerSubsystem scorer) {

    this.scorer = scorer;

  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    this.scorer.moveScorerToSafeCancelPosition();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
