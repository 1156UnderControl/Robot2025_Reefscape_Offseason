package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CoastState extends SequentialCommandGroup {
    OperatorController operatorController = OperatorController.getInstance();
    ScorerSubsystem scorer;
    IntakeSubsystem intake;

    public CoastState(ScorerSubsystem scorer, IntakeSubsystem intake){ 
        this.scorer = scorer;
        this.intake = intake;

        addCommands(
            new InstantCommand(() -> {
                this.scorer.setCoastMode();
                this.intake.setCoastMode();
            })
        );
    }
}