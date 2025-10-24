package frc.Java_Is_UnderControl.Swerve.IO.Chassi.Base;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface BaseSwerveSubsystemIO {
    
    @AutoLog
    public static class BaseSwerveSubsystemIOInputs {
        public ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        public ChassisSpeeds robotTargetSpeeds = new ChassisSpeeds();
        public double robotAngle = 0;
        public SwerveModuleState[] moduleState = new SwerveModuleState[4];
        public SwerveModuleState[] moduleTargetState = new SwerveModuleState[4];
    }
}
