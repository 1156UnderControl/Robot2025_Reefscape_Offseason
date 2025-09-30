package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.CollectClimber;
import frc.robot.commands.Scorer.MoveScorerToClimbPosition;
import frc.robot.commands.Swerve.AlignToCage;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class AlignToClimb extends SequentialCommandGroup {
    OperatorController operatorController = OperatorController.getInstance();
    ClimberSubsystem climber;
    ScorerSubsystem scorer;

    public AlignToClimb(ClimberSubsystem climber, SwerveSubsystem swerve, ScorerSubsystem scorer){ 
        this.climber = climber;
        this.scorer = scorer;
        addCommands(
            new MoveScorerToClimbPosition(scorer),
            new AlignToCage(swerve).alongWith(new CollectClimber(climber)).until(operatorController.climb()),
            new Climb(climber)
        );
    }
}