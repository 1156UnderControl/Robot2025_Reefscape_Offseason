package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.States.PrepareToScoreCoralState;
import frc.robot.commands.States.SwerveTeleopState;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final OperatorController operatorController;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;
  private final ScorerSubsystem scorer;
  private final IntakeSubsystem intake;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    this.scorer = ScorerSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();

    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.swerve.setDefaultCommand(Commands.run(() -> this.swerve.driveFieldOrientedLockedJoystickAngle(), this.swerve).onlyIf(() -> DriverStation.isTeleopEnabled()));

    this.configureButtonBindings();
  }

  private void configureButtonBindings() {

    this.driverController.b()
    .onTrue(new InstantCommand(() -> { 
      this.intake.goToIntakePosition();
    }));
    
    this.operatorController.goToReefA()
      .onTrue(new InstantCommand(() -> { 
        this.intake.goToIntakePosition();
      }));
    
    this.operatorController.reefL2()
      .onTrue(new InstantCommand(() -> { 
        this.scorer.setTargetCoralLevel(ReefLevel.L2);
        this.scorer.setManualScoreCoral(true);
      }));

    this.operatorController.reefL3()
      .onTrue(new InstantCommand(() -> { 
        this.scorer.setTargetCoralLevel(ReefLevel.L3);
        this.scorer.setManualScoreCoral(true);
      }));

    this.operatorController.reefL4()
      .onTrue(new InstantCommand(() -> { 
        this.scorer.setTargetCoralLevel(ReefLevel.L4);
        this.scorer.setManualScoreCoral(true);
      }));

    this.operatorController.prepareToScore()
      .onTrue(Commands.run(() -> this.scorer.prepareToScoreCoral(), scorer));

    this.driverController.a().onTrue(Commands.run(() -> this.scorer.prepareToScoreCoral(), this.scorer));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
