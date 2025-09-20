package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralFromIndexer extends Command{
    private ScorerSubsystem scorer;
    private Timer timer;

    public CollectCoralFromIndexer(ScorerSubsystem scorer){
        this.scorer = scorer;
        this.timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.start();
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
        this.timer.stop();
        this.timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.5;
    }
}
