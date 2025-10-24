package frc.Java_Is_UnderControl.Swerve.IO.Chassi.Odometry;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface OdometrySwerveSubsystemIO {
    @AutoLog
    public static class BaseSwerveSubsystemIOInputs {
        public ChassisSpeeds robotPose = new ChassisSpeeds();
        public ChassisSpeeds robotTargetPose = new ChassisSpeeds();
    }
} 
