package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double pivotPosition = 0.0;
        public double pivotTargetPosition = 0.0;
        public double cageIntakeVelocity = 0.0;
        public double cageIntakeTargetVelocity = 0.0;
        public boolean cageIntakeIsInverted = false;
        public boolean pivotIsInverted = false;
    }

    void setPivotDutyCicle(double dutyCycle);
    void setCageIntakeDutyCicle(double dutyCicle);
}
