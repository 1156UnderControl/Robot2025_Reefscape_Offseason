package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class OverrideCoralMode extends Command {
    private IntakeSubsystem intake;
    private boolean isActive;
      
    public OverrideCoralMode(IntakeSubsystem intake, boolean isActive) {
      this.intake = intake;
      this.isActive = isActive;
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
      this.intake.setOverrideCoralModeActive(this.isActive);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return this.intake.getOverrideCoralModeActive() == this.isActive;
    }
    }