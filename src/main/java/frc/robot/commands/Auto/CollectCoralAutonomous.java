package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralAutonomous extends Command {
  ScorerSubsystem scorer;
  IntakeSubsystem intake;

  public CollectCoralAutonomous(IntakeSubsystem intake, ScorerSubsystem scorer) {
    this.scorer = scorer;
    this.intake = intake;
    addRequirements(this.scorer, this.intake);
  }

  @Override
  public void initialize() {
    this.scorer.resetCollectCoralTimer();
  }

  @Override
  public void execute() {
    if(this.intake.indexerHasCoral()){
      this.scorer.collectCoralFromIndexer();
    } else {
      this.intake.goToIntakePosition();
      this.intake.collectCoral();
    }
  }

  @Override
  public boolean isFinished() {
    return this.scorer.getCollectCoralTimer() > 0.5 || this.scorer.hasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    this.scorer.overrideHasCoral();
    this.scorer.stopEndEffector();
    this.scorer.resetCollectCoralTimer();
  }
}
