package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;


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
  public void end(boolean interrupted) {
    this.intake.stopIntaking();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}