package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.Java_Is_UnderControl.Swerve.constants.SwerveConstants;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.commands.Scorer.CollectCoralFromIndexer;
import frc.robot.commands.Scorer.PrepareToScoreCoral;
import frc.robot.commands.Scorer.ScoreCoral;
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
  private final ScorerSubsystem scorer;

  private SwerveModuleConstants[] modulesArray = SwerveConstants.getModuleConstants();

  public RobotContainer() {
    this.scorer = ScorerSubsystem.getInstance();

    this.swerve = new SwerveSubsystem(this.scorer.getReefScoringModeSupplier(), this.scorer.getTargetCoralReefLevelSupplier(), this.scorer.getTargetAlgaeReefLevelSupplier(), SwerveConstants.getSwerveDrivetrainConstants(),
      modulesArray[0], modulesArray[1], modulesArray[2], modulesArray[3]);
   
    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    this.swerve.setDefaultCommand(Commands.run(() -> swerve.driveAlignAngleJoystick(), this.swerve));
    this.scorer.setDefaultCommand(Commands.run(() -> this.scorer.moveScorerToDefaultPosition(), this.scorer));
    this.configureButtonBindings();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {
    
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}