package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveTeleopState extends Command{

    SwerveSubsystem swerve;

    @Override
    public void initialize(){
        this.swerve = new SwerveSubsystem();
    }

    @Override
    public void execute(){
        this.swerve.driveFieldOrientedLockedJoystickAngle();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
