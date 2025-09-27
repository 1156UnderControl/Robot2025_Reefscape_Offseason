package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class CollectClimber extends Command{
    private ClimberSubsystem climber;

    public CollectClimber (ClimberSubsystem climber){
        this.climber = climber;
        this.addRequirements(this.climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.climber.goToPrepareClimbPosition();
        this.climber.startCollectingClimber();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
