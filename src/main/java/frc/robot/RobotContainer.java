package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.commands.Intake.CollectCoralFromIndexer;
import frc.robot.commands.Scorer.MoveScorerToScorePosition;
import frc.robot.commands.States.CollectCoralPosition;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final OperatorController operatorController;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final ScorerSubsystem scorer;

  private SwerveModuleConstants[] modulesArray = SwerveConstants.getModuleConstants();

  public RobotContainer() {
    this.scorer = ScorerSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();
    this.swerve = new SwerveSubsystem(this.scorer.getReefScoringModeSupplier(), this.scorer.getTargetCoralReefLevelSupplier(), this.scorer.getTargetAlgaeReefLevelSupplier(), SwerveConstants.getSwerveDrivetrainConstants(),
      modulesArray[0], modulesArray[1], modulesArray[2], modulesArray[3]);
   
    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    this.scorer.setIntakeUpSupplier(this.intake.getIntakeUpSupplier());
    this.swerve.setDefaultCommand(Commands.run(() -> swerve.driveAlignAngleJoystick(), this.swerve));
    this.scorer.setDefaultCommand(Commands.run(() -> this.scorer.moveScorerToDefaultPosition(), this.scorer));
    this.intake.setDefaultCommand(Commands.run(() -> this.intake.goToIntakePosition(), this.intake));
    this.configureButtonBindings();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {
    this.driverController.a().onTrue(
      new CollectCoralPosition(intake, scorer)
    );
    this.driverController.b().onTrue(
      Commands.run(() -> intake.setHasCoral())
    );

  }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}