package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbState extends Command {
      private ClimberSubsystem climber;

  public ClimbState (ClimberSubsystem climber){
      this.climber = ClimberSubsystem.getInstance();
      this.addRequirements(this.climber);}
@Override
public void initialize(){
      
}

@Override
public void execute(){
    this.climber.climb();
}

@Override
public boolean isFinished(){
 return this.climber.getIsAtClimbedPosition();
}
}