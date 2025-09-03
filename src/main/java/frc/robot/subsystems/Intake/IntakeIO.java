package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    void collectCoral();

    void goToDefaultPosition();

    boolean indexerHasCoral();

    
}
