package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class StopCollecting extends Command {
    private IntakeSubsystem intake;
      
    public StopCollecting(IntakeSubsystem intake) {
      this.intake = intake;
      this.addRequirements(this.intake);
      }
    
      @Override
      public void initialize() {}
    
      @Override
      public void execute() {
        this.intake.goToDefaultPosition();
        this.intake.stopIntaking();
      }
    
      @Override
      public void end(boolean interrupted) {}
    
      @Override
      public boolean isFinished() {
        return this.intake.isIntakeAtTargetPosition(IntakeConstants.tunning_values_intake.setpoints.INTAKE_ANGLE_HOMED);
      }
    }