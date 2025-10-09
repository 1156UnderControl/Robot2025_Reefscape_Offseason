package frc.Java_Is_UnderControl.Swerve.IO.Chassi.Base;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;

public interface BaseSwerveSubsystemIO {
    
    @AutoLog
    public static class BaseSwerveSubsystemIOInputs {
        public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
        public double absoluteTargetSpeed = 0;

        public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        public double absoluteMeasuredSpeed = 0;

        public double stdDevXY = 0;
        public double stdDevTheta = 0;

        public Pose2d currentPose = new Pose2d();
        public double targetHeadingDegrees = 0;
        public double measuredHeadingDegrees = 0;
        public boolean isAtTargetHeading = false;
    }

    Command applyRequest(Supplier<SwerveRequest> requestSupplier);

    void resetOdometry(Pose2d initialHolonomicPose);
    void resetTranslation(Translation2d translationToReset);
    void zeroGyro();
    void setHeadingCorrection(boolean active);
    void setMotorBrake(boolean brake);
    
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds);
    Supplier<SwerveRequest> lock();
}
