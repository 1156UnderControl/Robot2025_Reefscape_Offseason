package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class PreparedToIntakeCageState extends Command {
    private ClimberSubsystem climber;
    
public PreparedToIntakeCageState (ClimberSubsystem climber){
    this.climber = ClimberSubsystem.getInstance();
    this.addRequirements(this.climber);
}
@Override
public void initialize(){
    
    this.climber.unlockClimber();
}

@Override
public void execute(){
    this.climber.intakeCagePosition();
    if(this.climber.isPreparedToIntake()){
    this.climber.stopClimber();
    } 
 }

}
