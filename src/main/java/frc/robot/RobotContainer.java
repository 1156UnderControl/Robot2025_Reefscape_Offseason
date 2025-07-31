package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.States.SwerveTeleopState;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  private final SwerveSubsystem swerve;

  public RobotContainer() {
    this.swerve = new SwerveSubsystem();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    this.swerve.setDefaultCommand(new SwerveTeleopState());
    this.configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
