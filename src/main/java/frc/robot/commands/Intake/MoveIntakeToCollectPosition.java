package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class MoveIntakeToCollectPosition extends Command {
  private IntakeSubsystem intake;
  
    public MoveIntakeToCollectPosition(IntakeSubsystem intake) {
      this.intake = intake;
      this.addRequirements(this.intake);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    this.intake.goToIntakePosition();
    this.intake.collectCoral();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}