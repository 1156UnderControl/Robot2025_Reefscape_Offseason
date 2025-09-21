package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class CollectCoralPosition extends SequentialCommandGroup {

  public CollectCoralPosition(IntakeSubsystem intake, ScorerSubsystem scorer) {

    addCommands(
      //new MoveIntakeToCollectPosition(this.intake),
      new CollectCoralFromIndexer(scorer));
  }
}