package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoUpdateOdometry extends Command {
  SwerveSubsystem swerve;
  Translation2d defaultPostion;

  public AutoUpdateOdometry(SwerveSubsystem swerve, Translation2d defaultPosition) {
    this.swerve = swerve;
    this.defaultPostion = defaultPosition;
  }

  @Override
  public void initialize() {
    System.out.println("RESETANDO ODOMETRIA");
  }

  @Override
  public void execute() {
    this.swerve.resetOdometryLimelight(defaultPostion);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.positionUpdated();
  }

  @Override
  public void end(boolean interrupted) {

  }
}
