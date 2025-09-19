package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs{
        public boolean indexerHasCoral = false;
    }

    void collectCoral();

    void stopIntaking();

    void expellCoral();

    void goToDefaultPosition();

    void goToIntakePosition();

    boolean indexerHasCoral();

    boolean isIntakeAtTargetPosition(double targetPosition);

    Supplier<Boolean> getIntakeUpSupplier();

    void setCoastMode();

    void setBrakeMode();
}
