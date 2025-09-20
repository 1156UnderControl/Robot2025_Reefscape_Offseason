package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.commands.Scorer.MoveScorerToDefaultPosition;
import frc.robot.joysticks.DriverController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectCoralPosition extends SequentialCommandGroup {

  private IntakeSubsystem intake;
  private ScorerSubsystem scorer;
  private final DriverController driverController;

  public CollectCoralPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {
    this.intake = intake;
    this.scorer = scorer;
    this.driverController = DriverController.getInstance();

    addCommands(
      new MoveScorerToDefaultPosition(scorer),
      new MoveIntakeToCollectPosition(intake).until(this.driverController.b())
        .andThen(new ConditionalCommand(Commands.idle(scorer, intake), new CollectCoralFromIndexer(scorer), () -> this.scorer.hasCoral())));
  }
}