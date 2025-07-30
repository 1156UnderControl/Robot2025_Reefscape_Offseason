package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Joysticks.DriverController;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final DriverController driverController;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SwerveSubsystem swerve;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.driverController = DriverController.getInstance();
    this.swerve.setDefaultCommand(
        Commands.run(() -> this.swerve.driveFieldOrientedLockedJoystickAngle(), this.swerve));
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
