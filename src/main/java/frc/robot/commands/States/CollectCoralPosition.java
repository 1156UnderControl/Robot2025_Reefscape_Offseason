package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.joysticks.DriverController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class CollectCoralPosition extends SequentialCommandGroup {


  public CollectCoralPosition(IntakeSubsystem intake, ScorerSubsystem scorer, DriverController controller) {
    addCommands(
      new MoveIntakeToCollectPosition(intake),
      Commands.idle(scorer, intake).until(controller.a()),
      new CollectCoralFromIndexer(scorer, intake)
      );
  }
}