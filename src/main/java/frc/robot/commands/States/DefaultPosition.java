package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveIntakeToHomedPosition;
import frc.robot.commands.Scorer.MoveScorerToDefaultPosition;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class DefaultPosition extends SequentialCommandGroup{

  public DefaultPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {
    addCommands(
      new MoveScorerToDefaultPosition(scorer).alongWith(new MoveIntakeToHomedPosition(intake)));
  }
}