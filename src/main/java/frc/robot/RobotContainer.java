package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
  //private final IntakeSubsystem intake;
  private final ScorerSubsystem scorer;


  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    //this.intake = IntakeSubsystem.getInstance();
    this.scorer = ScorerSubsystem.getInstance();
   
    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    //this.swerve.setDefaultCommand(Commands.run(() -> swerve.driveFieldOrientedLockedJoystickAngle(), this.swerve));
    this.scorer.setDefaultCommand(Commands.run(() -> this.scorer.moveScorerToDefaultPosition(), this.scorer));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {
    //this.driverController.a()
    //.onTrue(
        //Commands.run(() -> this.scorer.moveScorerToDefaultPosition(), this.intake)
    //);
    
    //this.driverController.b()
    //.onTrue(
    //    Commands.run(() -> this.scorer.prepareToScoreCoral(), this.intake)
    //);
  }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}