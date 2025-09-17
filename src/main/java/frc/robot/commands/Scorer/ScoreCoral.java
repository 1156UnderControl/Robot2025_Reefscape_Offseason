package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ScoreCoral extends Command{
    private ScorerSubsystem scorer;

    @Override
    public void initialize(){
        this.scorer = ScorerSubsystem.getInstance();
        this.addRequirements(this.scorer);
    }

    @Override
    public void execute(){
        this.scorer.scoreCoral();
        if(this.scorer.isElevatorAtTargetPosition() && this.scorer.isPivotAtTargetPosition()){
        this.scorer.placeCoral();
    }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return this.scorer.isElevatorAtTargetPosition() && this.scorer.isPivotAtTargetPosition() && !this.scorer.hasCoral();
    }
}
