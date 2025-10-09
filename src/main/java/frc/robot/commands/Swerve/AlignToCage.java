package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignToCage extends Command{
    private SwerveSubsystem swerve;

    public AlignToCage (SwerveSubsystem swerve){
        this.swerve = swerve;
        this.addRequirements(this.swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.swerve.setAngleForClimb();
        this.swerve.driveLockedAngleToClimb();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
