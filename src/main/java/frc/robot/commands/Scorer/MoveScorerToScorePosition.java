package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class MoveScorerToScorePosition extends Command{
    private final ScorerSubsystem scorer;

    public MoveScorerToScorePosition(ScorerSubsystem scorer){
        this.scorer = scorer;
        this.addRequirements(this.scorer);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        this.scorer.moveToScoreCoral();
        if(this.scorer.isScorerAtTargetPosition()){
            this.scorer.placeCoral();
        }
    }
    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return this.scorer.isScorerAtTargetPosition();
    }
}
