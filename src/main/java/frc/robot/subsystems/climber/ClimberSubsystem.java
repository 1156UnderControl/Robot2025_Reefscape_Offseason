package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
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
    private double goalPivot;
    private boolean stopClimber = false;
    private double LimitPosition;

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
          instance = new ClimberSubsystem();
        }
        return instance;
    }

    private ClimberSubsystem() {
        this.climberInputs = new ClimberIOInputsAutoLogged();
        this.pivotInputs = new MotorIOInputsAutoLogged();
        this.cageIntakeInputs = new MotorIOInputsAutoLogged();

        this.pivotMotor = new TalonFXMotor(14, 1, GravityTypeValue.Arm_Cosine, "Pivot Climber Motor");
        this.cageIntakeMotor = new SparkMAXMotor(2, 1, "Cage Intake Motor");

        this.configureClimberMotor();
    }
    private void configureClimberMotor() {
        pivotMotor.setInverted(false);
        pivotMotor.setPositionFactor(250);
        pivotMotor.configurePIDF(
            ClimberConstants.setpoints.PID.P,
            ClimberConstants.setpoints.PID.I,
            ClimberConstants.setpoints.PID.D, 0);
       pivotMotor.setMinMotorOutput(-0.75);
        cageIntakeMotor.burnFlash();
        pivotMotor.setMotorBrake(true);
        pivotMotor.setPosition(0);
      }
    

    @Override
    public void periodic(){
        this.updateLogs(climberInputs);
    }

    private void updateLogs(ClimberIOInputsAutoLogged climberInputs) {
        this.pivotMotor.updateInputs(pivotInputs);
        this.cageIntakeMotor.updateInputs(cageIntakeInputs);

        Logger.processInputs("Motors/Climber/Pivot", pivotInputs);
        Logger.processInputs("Motors/Climber/cageIntake", cageIntakeInputs);
    }
    
    public boolean isAtClimbPosition(){
        if (pivotMotor.setPositionReference(ClimberConstants.setpoints.GOAL_PREPARE_TO_CLIMB)) {
        return true; } else{ return false;
        
    }
}
    public void climb(){
        pivotMotor.setPositionReference(ClimberConstants.setpoints.CLIMB);
    }
    
    public double limitPosition(){
        if (goalPivot >= ClimberConstants.setpoints.LIMIT_POSITION);
        return (goalPivot > ClimberConstants.setpoints.MAX_ANGLE)?
        ClimberConstants.setpoints.MAX_ANGLE:
        (goalPivot<ClimberConstants.setpoints.MIN_ANGLE?
        ClimberConstants.setpoints.MIN_ANGLE: goalPivot);
    }


    public boolean isPreparedToIntake(){
    return Math.abs(
    cageIntakeMotor.getPosition()-
    ClimberConstants.setpoints.MIN_ANGLE) < 0.01;
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
