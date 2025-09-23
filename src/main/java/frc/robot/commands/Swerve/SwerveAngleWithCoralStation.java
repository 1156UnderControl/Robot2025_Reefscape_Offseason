package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveAngleWithCoralStation extends Command {
  ISwerve swerve;

  public SwerveAngleWithCoralStation(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.swerve.driveLockedAngleToNearestCoralStation();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
