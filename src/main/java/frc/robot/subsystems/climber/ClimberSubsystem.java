package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.SparkMAXMotor;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.robot.constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements ClimberIO {
    private static ClimberSubsystem instance;

    private final ClimberIOInputsAutoLogged climberInputs;
    private final MotorIOInputsAutoLogged pivotInputs;
    private final MotorIOInputsAutoLogged cageIntakeInputs;

    private final MotorIO pivotMotor;
    private final MotorIO cageIntakeMotor;
    private final double goal;
    private final double stopClimber = false;

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

        this.pivotMotor = new TalonFXMotor(14, 1, GravityTypeValue.Arm_Cosine, "Pivot Climber Motor");
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
    private void configureClimberMotor() {
        pivotMotor.setInverted(false);
        pivotMotor.setPositionFactor(250);
        pivotMotor.configurePIDF(
            ClimberConstants.tunning_values_arm.PID.P,
            ClimberConstants.tunning_values_arm.PID.I,
            ClimberConstants.tunning_values_arm.PID.D, 0);
       pivotMotor.setMinMotorOutput(-0.75);
        cageIntakeMotor.burnFlash();
        pivotMotor.setMotorBrake(true);
        pivotMotor.setPosition(0);
      }
    
    public boolean isAtClimbPosition(){
        if ( pivotMotor.setPositionReference(ClimberConstants.setpoints.GOAL_PREPARE_TO_CLIMB)) {
        return true; } else{ return false;
        
    }
}
    public void climb(){
        pivotMotor.setPositionReference(ClimberConstants.setpoints.CLIMB);
    }
    
    public double limitPosition(){
        double goal = limitPosition(ClimberConstants.setpoints.LIMIT_POSITION);
        return (goal > ClimberConstants.setpoints.MAX_ANGLE)?
        ClimberConstants.setpoints.MAX_ANGLE:
        (goal<ClimberConstants.setpoints.MIN_ANGLE?
        ClimberConstants.setpoints.MIN_ANGLE: goal);
    }


    public boolean isPreparedToIntake(){
       return Util.atSetpoint(pivotMotor.getPosition(),
       ClimberConstants.setpoints.MIN_ANGLE, 0.01);
    }
    public void intakeCagePosition(){
        if (stopClimber){
            pivotMotor.set(0); }else{
                pivotMotor.setPositionReference(ClimberConstants.setpoints.INTAKE_CAGE_ANGLE);
            }
        }

    public void setCoastClimber(){
    pivotMotor.setMotorBrake(false);
    cageIntakeMotor.setMotorBrake(false);
    }

    public void setBrakeClimber(){
        pivotMotor.setMotorBrake(true);
        pivotMotor.setMotorBrake(true);
    }
    @Override
    public void setCageIntakeDutyCicle(double dutyCicle) {
        cageIntakeMotor.set(dutyCicle);
    }
    public void lockClimber(){
    pivotMotor.set(1);
    }

    public void unlockClimber(){
        pivotMotor.set(0);
    }

    @Override
    public void setPivotDutyCicle(double dutyCycle) {
        pivotMotor.set(dutyCycle);
    }
}
