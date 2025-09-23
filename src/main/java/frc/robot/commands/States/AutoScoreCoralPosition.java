package frc.robot.commands.States;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();
  IDriverController driverController = DriverController.getInstance();
  boolean hasCancelledAutoMove = false;
  boolean driverHasCancelledAutoMove = false;

  public AutoScoreCoralPosition(ScorerSubsystem scorer, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new InstantCommand(() -> {
      hasCancelledAutoMove = false;
      driverHasCancelledAutoMove = false;
    }),
        new GoAndRaiseElevator(swerve, scorer, branch)
            .until(new Trigger(() -> {
              if (driverController.isForcingDriverControl().getAsBoolean()) {
                driverHasCancelledAutoMove = true;
              }
              return driverHasCancelledAutoMove;
            }).or(() -> {
              if (operatorKeyboard.scoreObject().getAsBoolean()) {
                hasCancelledAutoMove = true;
              }
              return hasCancelledAutoMove;
            })),
        new ParallelRaceGroup(new MoveScorerToPrepareScore(scorer)
            .until(operatorKeyboard.scoreObject()
                .or(() -> (swerve.isAtTargetPositionWithoutHeading() && scorer.isScorerAtTargetPosition())
                    && !driverHasCancelledAutoMove)
                .or(() -> hasCancelledAutoMove)),
            Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve)),
        new MoveScorerToScorePosition(scorer),
        Commands.waitTime(Seconds.of(0.3)),
        Commands.idle(scorer).alongWith(Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve))
            .until(operatorKeyboard.removeAlgaeFromBranch())
       );
  }
}
