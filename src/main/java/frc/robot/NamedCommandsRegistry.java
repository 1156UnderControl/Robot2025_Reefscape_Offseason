package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.robot.commands.Auto.AutoScoreCoralAutonomous;
import frc.robot.commands.Auto.AutoUpdateOdometry;
import frc.robot.commands.Auto.CollectCoralAutonomous;
import frc.robot.commands.States.DefaultPosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NamedCommandsRegistry {
  ScorerSubsystem scorer;
  SwerveSubsystem drivetrain;
  IntakeSubsystem intake;

  public NamedCommandsRegistry(SwerveSubsystem drivetrain,  ScorerSubsystem scorer, IntakeSubsystem intake) {
    this.scorer = scorer;
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  public void registerAllAutoCommands() {
    this.registerResetOdometryCommands(this.drivetrain);
    this.registerIntakeCommands(this.scorer);
    this.registerCoralScoringCommands(this.intake, this.scorer, this.drivetrain);
    this.registerLevelCommands(this.scorer);
  }

  private void registerResetOdometryCommands(SwerveSubsystem drivetrain) {
    NamedCommands.registerCommand("ResetOdometry Left",
        new InstantCommand(() -> drivetrain.resetOdometry(
            AllianceFlipUtil.apply(new Pose2d(7.18, 4.730, Rotation2d.k180deg)))));
    NamedCommands.registerCommand("ResetOdometry Right",
        new InstantCommand(() -> drivetrain.resetOdometry(
            AllianceFlipUtil.apply(new Pose2d(7.18, 3.317, Rotation2d.k180deg)))));

    NamedCommands.registerCommand("ResetOdometry Left XY",
        new InstantCommand(() -> drivetrain.resetTranslation(
            AllianceFlipUtil.apply(new Translation2d(7.18, 4.730)))));
    NamedCommands.registerCommand("ResetOdometry Right XY",
        new InstantCommand(() -> drivetrain.resetTranslation(
            AllianceFlipUtil.apply(new Translation2d(7.18, 3.317)))));

    NamedCommands.registerCommand("ResetOdometry Center XY With Vision",
        new AutoUpdateOdometry(drivetrain, AllianceFlipUtil.apply(new Translation2d(7.18, 4))));
    NamedCommands.registerCommand("ResetOdometry Right XY With Vision",
        new AutoUpdateOdometry(drivetrain, AllianceFlipUtil.apply(new Translation2d(7.18, 3.317))));
    NamedCommands.registerCommand("ResetOdometry Left XY With Vision",
        new AutoUpdateOdometry(drivetrain, AllianceFlipUtil.apply(new Translation2d(7.18, 4.730))));
  }

  private void registerIntakeCommands(ScorerSubsystem scorer) {
    NamedCommands.registerCommand("Intake Coral", new CollectCoralAutonomous(intake, scorer));
  }

  private void registerCoralScoringCommands(IntakeSubsystem intake, ScorerSubsystem scorer, SwerveSubsystem drivetrain) {
    for (TargetBranch branch : TargetBranch.values()) {
      String name = branch.name();
      NamedCommands.registerCommand("Score Coral " + name,
          new AutoScoreCoralAutonomous(scorer, intake, drivetrain, branch));
    }
  }

  private void registerLevelCommands(ScorerSubsystem scorer) {
    for (ReefLevel level : ReefLevel.values()) {
      if (level != ReefLevel.TO_L4) {
        NamedCommands.registerCommand("Set Coral Level " + level.name(),
            new InstantCommand(() -> scorer.setTargetCoralLevel(level)));
      }
    }
  }
}
