package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.commands.Scorer.MoveScorerToPrepareScore;
import frc.robot.commands.Scorer.UpdatePivotInternalEncoder;
import frc.robot.commands.States.CollectCoralPosition;
import frc.robot.commands.States.DefaultPosition;
import frc.robot.commands.States.ScoreObjectPosition;
import frc.robot.commands.States.BrakeState;
import frc.robot.commands.States.CoastState;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
    this.scorer.setDefaultCommand(new DefaultPosition(intake, scorer));
    this.intake.setDefaultCommand(Commands.run(() -> this.intake.goToIntakePosition(), this.intake));
    this.configureButtonBindings();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureButtonBindings() {
    
    this.driverController.y().onTrue(
      new CollectCoralPosition(intake, scorer)
    );

    this.driverController.x().and(() -> scorer.hasObject()).onTrue(
      new ScoreObjectPosition(scorer)
    );

    this.driverController.leftBumper().onTrue(new DefaultPosition(intake, scorer));


    this.driverController.leftArrow().onTrue(
      new InstantCommand(() -> this.scorer.setTargetCoralLevel(ReefLevel.L1))
    );

    this.driverController.rightArrow().onTrue(
      new InstantCommand(() -> this.scorer.setTargetCoralLevel(ReefLevel.L2))
    );

    this.driverController.downArrow().onTrue(
      new InstantCommand(() -> this.scorer.setTargetCoralLevel(ReefLevel.L3))
    );

    this.driverController.upArrow().onTrue(
      new InstantCommand(() -> this.scorer.setTargetCoralLevel(ReefLevel.L4))
    );

    driverController.x().and(() -> DriverStation.isDisabled()).whileTrue(Commands
        .runEnd(() -> new CoastState(scorer, intake), () -> new BrakeState(scorer, intake)).ignoringDisable(true));

    driverController.y().and(() -> DriverStation.isDisabled()).whileTrue(new UpdatePivotInternalEncoder(scorer).ignoringDisable(true));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}