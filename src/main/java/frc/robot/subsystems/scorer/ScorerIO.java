package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.AutoLog;

public interface ScorerIO {

    @AutoLog
    public static class ScorerIOInputs{
        public double elevatorLeadPosition = 0.0;
        public double elevatorLeadTargetPosition = 0.0;
        public double elevatorLeadVelocity = 0.0;
        public double elevatorLeadTargetVelocity = 0.0;
        public boolean elevatorLeadIsInverted = false;

        public double elevatorFollowerPosition = 0.0;
        public double elevatorFollowerTargetPosition = 0.0;
        public double elevatorFollowerVelocity = 0.0;
        public double elevatorFollowerTargetVelocity = 0.0;
        public boolean elevatorFollowerIsInverted = false;
    }

    void setElevatorDutyCicle(double dutyCycle);
}
