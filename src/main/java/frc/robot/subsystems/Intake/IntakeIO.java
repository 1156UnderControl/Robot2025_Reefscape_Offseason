package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

public interface IntakeIO {

    void collectCoral();

    void stopIntaking();

    void expellCoral();

    void goToDefaultPosition();

    void goToIntakePosition();

    boolean indexerHasCoral();

    boolean isIntakeAtTargetPosition(double targetPosition);

    Supplier<Boolean> getIntakeUpSupplier();
}
