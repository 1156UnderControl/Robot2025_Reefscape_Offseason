package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final OperatorController operatorController;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;


  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    this.intake = IntakeSubsystem.getInstance();
   
    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.swerve.setDefaultCommand(Commands.run(() -> this.swerve.driveFieldOrientedLockedJoystickAngle(), this.swerve).onlyIf(() -> DriverStation.isTeleopEnabled()));
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.driverController.a()
    .onTrue(
        Commands.run(() -> this.intake.collectCoral(), this.intake)
    );
    
    this.driverController.b()
    .onTrue(
        Commands.run(() -> this.intake.stopIntaking(), this.intake)
    );
  }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}