package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbState extends Command {
      private ClimberSubsystem climber;

@Override
public void initialize(){
    this.climber = ClimberSubsystem.getInstance();
    this.addRequirements(this.climber);
   
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