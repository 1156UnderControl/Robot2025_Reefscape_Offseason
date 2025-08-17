package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Joysticks.DriverController;
import frc.robot.commands.States.SwerveTeleopState;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;
  private final ScorerSubsystem scorer;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    this.scorer = ScorerSubsystem.getInstance();
    this.driverController = DriverController.getInstance();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.swerve.setDefaultCommand(Commands.run(() -> new SwerveTeleopState(), this.swerve));
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.driverController.a().onTrue(Commands.run(() -> scorer.setElevatorDutyCicle(0.1), scorer))
        .onFalse(Commands.run(() -> scorer.setElevatorDutyCicle(0), scorer));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
