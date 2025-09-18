package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.PrepareToScoreCoral;
import frc.robot.commands.Scorer.ScoreCoral;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class ManualScoreCoralState extends SequentialCommandGroup{

    private final ScorerSubsystem scorer;

    public ManualScoreCoralState(ScorerSubsystem scorer, OperatorController controller){
        this.scorer = scorer;
        this.addCommands(
            new PrepareToScoreCoral(this.scorer),
            new ScoreCoral(this.scorer).onlyIf(controller.scoreObject())
        );
    }
}
