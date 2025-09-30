package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class MoveScorerToClimbPosition extends Command{
    private final ScorerSubsystem scorer;

    public MoveScorerToClimbPosition(ScorerSubsystem scorer){
        this.scorer = scorer;
        this.addRequirements(this.scorer);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        this.scorer.moveScorerToClimbPosition();
    }

    @Override
    public void end(boolean interrupted) {
        this.scorer.setGoingToClimbElevatorPositionFalse();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
