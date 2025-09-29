package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralFromIndexer extends Command{
    private ScorerSubsystem scorer;
    private IntakeSubsystem intake;

    public CollectCoralFromIndexer(ScorerSubsystem scorer, IntakeSubsystem intake){
        this.scorer = scorer;
        this.intake = intake;
        addRequirements(this.scorer);
    }

    @Override
    public void initialize(){
        this.addRequirements(this.scorer);
        this.scorer.resetCollectCoralTimer();
    }

    @Override
    public void execute(){
        this.scorer.collectCoralFromIndexer();
    }

    @Override
    public void end(boolean interrupted) {
        this.scorer.stopEndEffector();
        this.scorer.overrideHasCoral();
        this.scorer.resetCollectCoralTimer();
        this.intake.setOverrideCoralModeActive(false);
    }

    @Override
    public boolean isFinished() {
        return this.scorer.getCollectCoralTimer() > 0.5;
    }
}
