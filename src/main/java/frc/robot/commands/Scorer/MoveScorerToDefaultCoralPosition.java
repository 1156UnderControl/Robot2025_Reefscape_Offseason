package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class MoveScorerToDefaultCoralPosition extends Command {
  
  private ScorerSubsystem scorer;
  boolean wentToSafe = false;
  boolean finished = false;

  public MoveScorerToDefaultCoralPosition(ScorerSubsystem scorer) {
    this.scorer = scorer;
    this.addRequirements(scorer);
  }


  @Override
  public void initialize() {
    wentToSafe = false;
    finished = false;
  }


  @Override
  public void execute() {
    if(wentToSafe){
      this.scorer.movePivotToDefaultWithGP();
      if(this.scorer.isPivotAtTargetPosition()){
        this.scorer.moveElevatorToDefaultWithCoral();
        finished = true;
      }
    }else{
      this.scorer.moveElevatorToTransitionDefault();
      if(this.scorer.isElevatorAtTargetPosition()){
        this.wentToSafe = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    
  }


  @Override
  public boolean isFinished() {
    return this.scorer.isPivotAtTargetPosition() && this.scorer.isElevatorAtTargetPosition() && finished;
  }
}
