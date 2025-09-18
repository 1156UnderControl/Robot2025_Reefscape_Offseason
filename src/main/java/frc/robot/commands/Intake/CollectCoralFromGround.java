package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CollectCoralFromGround extends Command{
    private IntakeSubsystem intake;

    public CollectCoralFromGround(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        this.addRequirements(this.intake);
    }

    @Override
    public void execute(){
        this.intake.collectCoral();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return this.intake.indexerHasCoral();
    }
}
