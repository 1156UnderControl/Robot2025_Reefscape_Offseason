package frc.robot.commands.States;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.commands.Swerve.SwerveGoToBackupIfNecessary;
import frc.robot.commands.Swerve.SwerveGoToBranch;
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
    addCommands(
    new SwerveGoToBackupIfNecessary(swerve, branch),
    new SwerveGoToBranch(swerve, branch));
  }
}
