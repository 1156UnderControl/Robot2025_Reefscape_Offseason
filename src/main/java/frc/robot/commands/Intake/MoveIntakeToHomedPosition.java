

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;


public class MoveIntakeToHomedPosition extends Command {
private IntakeSubsystem intake;
  
public MoveIntakeToHomedPosition(IntakeSubsystem intake) {
  this.intake = intake;
  this.addRequirements(this.intake);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.goToDefaultPosition();
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
