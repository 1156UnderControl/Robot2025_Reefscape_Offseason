package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class MoveScorerToScorePosition extends Command{
    private final ScorerSubsystem scorer;
    boolean placedCoral = false;

    public MoveScorerToScorePosition(ScorerSubsystem scorer){
        this.scorer = scorer;
        this.addRequirements(this.scorer);
    }

    @Override
    public void initialize(){
        placedCoral = false;
    }

    @Override
    public void execute(){
        this.scorer.moveToScoreCoral();
        if(this.scorer.isScorerAtTargetPosition()){
            this.scorer.placeCoral();
            this.placedCoral = true;
        }
    }
    
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return placedCoral;
    }
}
