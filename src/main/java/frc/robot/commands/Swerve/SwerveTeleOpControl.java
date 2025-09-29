package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveTeleOpControl extends Command {
  private ISwerve swerve;

  public SwerveTeleOpControl(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    swerve.driveAlignAngleJoystick();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
