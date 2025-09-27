package frc.robot.subsystems.climber;

public interface ClimberIO {
    void goToClimbedPosition();
    void goToDefaultPosition();
    void goToPrepareClimbPosition();
    boolean isPreparedToIntake();
    void setCoastClimber();
    void setBrakeClimber();
    void startCollectingClimber();
    void stopCollectingClimber();
}
