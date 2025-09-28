package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class ExpellAlgae extends Command {
private ScorerSubsystem scorer;
  public ExpellAlgae(ScorerSubsystem scorer) {

this.scorer = scorer;

addRequirements(this.scorer);

  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.scorer.expellAlgae();
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
