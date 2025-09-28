package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralAutonomous extends Command {
  ScorerSubsystem scorer;
  IntakeSubsystem intake;
  boolean seenCoralInIndexer;
  boolean stopWithTimer;
  Timer collectTimer;

  public CollectCoralAutonomous(IntakeSubsystem intake, ScorerSubsystem scorer, boolean stopWithTimer) {
    this.scorer = scorer;
    this.intake = intake;
    this.seenCoralInIndexer = false;
    this.stopWithTimer = stopWithTimer;
    addRequirements(this.scorer, this.intake);
    collectTimer = new Timer();
  }

  @Override
  public void initialize() {
    this.seenCoralInIndexer = false;
    this.intake.goToIntakePosition();
    this.intake.collectCoral();
    this.scorer.resetCollectTimer();
    collectTimer.restart();
  }

  @Override
  public void execute() {
    if(this.intake.indexerHasCoral()){
      this.seenCoralInIndexer = true;
    }
    if(this.seenCoralInIndexer){
      this.scorer.collectCoralFromIndexer();
      if(this.scorer.getCollectTimer() > 0.5){
        this.scorer.overrideHasCoral();
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (collectTimer.get() >= 0.4 || this.stopWithTimer) {
      return true;
    }
    return scorer.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    collectTimer.stop();
    collectTimer.reset();
    if(this.scorer.hasCoral()){
      this.scorer.stopEndEffector();
      this.scorer.resetCollectTimer();
    }
  }
}
