package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements ClimberIO {
    private static ClimberSubsystem instance;

    private final ClimberIOInputsAutoLogged climberInputs;
    private final MotorIOInputsAutoLogged pivotInputs;
    private final MotorIOInputsAutoLogged cageIntakeInputs;

    private final MotorIO pivotMotor;
    private final MotorIO cageIntakeMotor;

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
          instance = new ClimberSubsystem();
        }
        return instance;
    }

    private ClimberSubsystem(){
        this.climberInputs = new ClimberIOInputsAutoLogged();
        this.pivotInputs = new MotorIOInputsAutoLogged();
        this.cageIntakeInputs = new MotorIOInputsAutoLogged();

        this.pivotMotor = new TalonFXMotor(14, "can_s1", GravityTypeValue.Arm_Cosine, "Pivot Climber Motor");
        this.cageIntakeMotor = new SparkMAXMotor(2, 1, "Cage Intake Motor");
    }

    @Override
    public void periodic(){
        this.updateClimberInputs(climberInputs);
    }

    private void updateClimberInputs(ClimberIOInputsAutoLogged climberInputs) {
        climberInputs.pivotPosition = pivotMotor.getPosition();
        climberInputs.pivotTargetPosition = climberInputs.pivotTargetPosition;
        climberInputs.cageIntakeVelocity = climberInputs.cageIntakeVelocity;
        climberInputs.cageIntakeTargetVelocity = climberInputs.cageIntakeTargetVelocity;
        climberInputs.pivotIsInverted = climberInputs.pivotIsInverted;
        climberInputs.cageIntakeIsInverted = climberInputs.cageIntakeIsInverted;

        this.pivotMotor.updateInputs(pivotInputs);
        this.cageIntakeMotor.updateInputs(cageIntakeInputs);

        Logger.processInputs("Subsystems/Climber/", climberInputs);
    }

    @Override
    public void setCageIntakeDutyCicle(double dutyCicle) {
        cageIntakeMotor.set(dutyCicle);
    }

    @Override
    public void setPivotDutyCicle(double dutyCycle) {
        pivotMotor.set(dutyCycle);
    }
}
