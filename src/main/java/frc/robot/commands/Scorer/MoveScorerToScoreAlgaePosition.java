package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveScorerToScoreAlgaePosition extends Command {
  
  private ScorerSubsystem scorer;

  public MoveScorerToScoreAlgaePosition() {
    
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
