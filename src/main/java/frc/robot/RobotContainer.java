package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Joysticks.DriverController;
import frc.robot.commands.States.SwerveTeleopState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;
  private final ScorerSubsystem scorer;
  private final ClimberSubsystem climber;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    this.scorer = ScorerSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.driverController = DriverController.getInstance();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.swerve.setDefaultCommand(Commands.run(() -> new SwerveTeleopState(), this.swerve));
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {
    this.driverController.a().onTrue(Commands.run(() -> climber.setPivotDutyCicle(1)))
        .onFalse(Commands.run(() -> climber.setPivotDutyCicle(0)));

    this.driverController.y().onTrue(Commands.run(() -> climber.setPivotDutyCicle(-1)))
        .onFalse(Commands.run(() -> climber.setPivotDutyCicle(0)));

    this.driverController.b().onTrue(Commands.run(() -> climber.setCageIntakeDutyCicle(1)))
        .onFalse(Commands.run(() -> climber.setCageIntakeDutyCicle(0)));

    this.driverController.y().onTrue(Commands.run(() -> climber.setCageIntakeDutyCicle(-1)))
        .onFalse(Commands.run(() -> climber.setCageIntakeDutyCicle(0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
