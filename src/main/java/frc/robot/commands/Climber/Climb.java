package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class Climb extends Command{
    private ClimberSubsystem climber;

    public Climb (ClimberSubsystem climber){
        this.climber = climber;
        this.addRequirements(this.climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.climber.stopCollectingClimber();
        this.climber.goToClimbedPosition();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
