package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class PrepareToScoreCoral extends Command{
    private final ScorerSubsystem scorer;

    public PrepareToScoreCoral(ScorerSubsystem scorer){
        this.scorer = scorer;
        this.addRequirements(this.scorer);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(this.scorer.hasCoral()){
            this.scorer.moveToPrepareScoreCoral();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
