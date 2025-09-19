package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class MoveScorerToDefaultPosition extends Command {
  
  private ScorerSubsystem scorer;

  public MoveScorerToDefaultPosition(ScorerSubsystem scorer) {

  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    this.scorer.moveScorerToDefaultPosition();
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return this.scorer.isPivotAtTargetPosition() && this.scorer.isElevatorAtTargetPosition();
  }
}
