package frc.robot.subsystems.Intake;

public interface IntakeIO {

    void collectCoral();

    void stopIntaking();

    void expellCoral();

    void goToDefaultPosition();

    void goToIntakePosition();

    boolean indexerHasCoral();
}
