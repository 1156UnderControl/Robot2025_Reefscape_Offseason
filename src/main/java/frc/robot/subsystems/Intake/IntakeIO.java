package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    void collectCoralIntake();
    
    void collectCoralIndexer();

    void goToDefaultPosition();

    boolean hasCollected();

    boolean isAtSetPoint();
    
}
