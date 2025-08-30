package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.States.PrepareToScoreCoralState;
import frc.robot.commands.States.SwerveTeleopState;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.scorer.ScorerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final OperatorController operatorController;
  private final DriverController driverController;

  private final SwerveSubsystem swerve;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();

    this.driverController = DriverController.getInstance();
    this.operatorController = OperatorController.getInstance();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    this.swerve.setDefaultCommand(Commands.run(() -> new SwerveTeleopState(), this.swerve));
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
