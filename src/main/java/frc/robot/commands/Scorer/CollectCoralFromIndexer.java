package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralFromIndexer extends Command{
    private ScorerSubsystem scorer;

    public CollectCoralFromIndexer(ScorerSubsystem scorer){
        this.scorer = scorer;
    }

    @Override
    public void initialize(){
        this.addRequirements(this.scorer);
    }

    @Override
    public void execute(){
        this.scorer.collectCoralFromIndexer();
    }

    @Override
    public void end(boolean interrupted) {
        this.scorer.stopEndEffector();
        this.scorer.overrideHasCoral();
        this.scorer.resetCollectTimer();
    }

    @Override
    public boolean isFinished() {
        return this.scorer.getCollectTimer() > 0.5;
    }
}
