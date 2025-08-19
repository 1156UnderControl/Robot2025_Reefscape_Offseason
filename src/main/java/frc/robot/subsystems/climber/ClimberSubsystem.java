package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.robot.subsystems.scorer.ScorerIOInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
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
        
    }

    private void updateClimberInputs(ClimberIOInputsAutoLogged climberInputs) {
        climberInputs.pivotPosition = pivotMotor.getPosition();
        climberInputs.pivotTargetPosition = pivotMotor.;
        climberInputs.cageIntakeVelocity = pivotInputs.velocity;
        climberInputs.cageIntakeTargetVelocity = pivotInputs.targetSpeed;
        climberInputs.pivotIsInverted = pivotMotor.isInverted;
        climberInputs.cageIntakeIsInverted = cageIntakeInputs.isInverted;

        Logger.processInputs("Subsystems/Climber/", climberInputs);
    }
}
