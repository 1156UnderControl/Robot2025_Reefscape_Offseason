package frc.robot.subsystems.climber;

public interface ClimberIO {
    void goToClimbedPosition();
    void goToDefaultPosition();
    void goToPrepareClimbPosition();
    boolean isPreparedToIntake();
    void setPivotDutyCicle(double dutyCycle);
    void setCageIntakeDutyCicle(double dutyCicle);
    void setCoastClimber();
    void setBrakeClimber();
    void stopClimber();
}
