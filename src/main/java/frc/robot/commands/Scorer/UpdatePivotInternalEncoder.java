package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class UpdatePivotInternalEncoder extends Command{
    private ScorerSubsystem scorer;
  boolean wentToSafe = false;
  boolean finished = false;

  public UpdatePivotInternalEncoder(ScorerSubsystem scorer) {
    this.scorer = scorer;
    this.addRequirements(scorer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(this.scorer.isUpdatingInternalPivotEncoderNecessary()){
        this.scorer.updateInternalPivotEncoder();
    }
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
