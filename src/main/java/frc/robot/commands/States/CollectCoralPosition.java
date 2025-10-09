package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToCollectPosition;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CollectCoralPosition extends SequentialCommandGroup {

  public CollectCoralPosition(IntakeSubsystem intake) {
    addCommands(
      new MoveIntakeToCollectPosition(intake)
      );
  }
}