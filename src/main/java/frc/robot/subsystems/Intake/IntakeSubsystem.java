package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.Java_Is_UnderControl.Sensors.SensorIO;
import frc.robot.constants.IntakeConstants;
import frc.robot.joysticks.OperatorController;

public class IntakeSubsystem extends SubsystemBase implements IntakeIO{
    private static IntakeSubsystem instance;
    private final XboxController controller = new XboxController(0);

    private final MotorIO intakeWheels;
    private final MotorIO intakePivot;
    private final MotorIO indexer;

    private final SensorIO indexerSensor;
    
    private final MotorIOInputsAutoLogged intakeWheelsInputs;
    private final MotorIOInputsAutoLogged intakePivotInputs;
    private final MotorIOInputsAutoLogged indexerInputs;
    private final IntakeIOInputsAutoLogged intakeInputs;

    private boolean indexerHasCoral;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
          instance = new IntakeSubsystem();
        }
        return instance;
    }

    private IntakeSubsystem(){
        this.intakeWheels = new SparkFlexMotor(IntakeConstants.ID_intakeWheelsMotor, IntakeConstants.intakeBusID, IntakeConstants.intakeWheelsMotorName);
        this.intakePivot = new SparkFlexMotor(IntakeConstants.ID_intakePivotMotor, IntakeConstants.intakeBusID, IntakeConstants.intakePivotMotorName);
        this.indexer = new SparkFlexMotor(IntakeConstants.ID_indexerMotor, IntakeConstants.intakeBusID, IntakeConstants.indexerMotorName);

        this.indexerSensor = new InfraRed(0, false);

        this.intakeWheelsInputs = new MotorIOInputsAutoLogged();
        this.intakePivotInputs = new MotorIOInputsAutoLogged();
        this.indexerInputs = new MotorIOInputsAutoLogged();
        this.intakeInputs = new IntakeIOInputsAutoLogged();
        this.indexerHasCoral = false;

        this.setConfigsIntakePivot();
    }

    private void updateLogs(){
        this.intakeInputs.indexerHasCoral = this.indexerHasCoral;
        this.intakeWheels.updateInputs(intakeWheelsInputs); 
        this.intakePivot.updateInputs(intakePivotInputs);
        this.indexer.updateInputs(indexerInputs);

        Logger.processInputs("Subsystems/Intake/", intakeInputs);
        Logger.processInputs("Motors/Intake/Wheels", intakeWheelsInputs);
        Logger.processInputs("Motors/Intake/Pivot", intakePivotInputs);
        Logger.processInputs("Motors/Intake/Indexer", indexerInputs);
    }

    private void setConfigsIntakePivot() {
        this.intakePivot.setMotorBrake(true);
        this.intakePivot.setInverted(false);
        this.intakePivot.setPositionFactor(IntakeConstants.tunning_values_intake.setpoints.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_ANGLE);
        this.intakePivot.setVelocityFactor(IntakeConstants.tunning_values_intake.setpoints.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_ANGLE_PER_SECOND);
        this.intakePivot.configurePIDF(
            IntakeConstants.tunning_values_intake.PID.P,
            IntakeConstants.tunning_values_intake.PID.I,
            IntakeConstants.tunning_values_intake.PID.D,
            IntakeConstants.tunning_values_intake.PID.arbFF,
            IntakeConstants.tunning_values_intake.PID.IZone);
        this.intakePivot.burnFlash();
        this.intakePivot.setPosition(IntakeConstants.tunning_values_intake.setpoints.ZERO_POSITION_IN_ANGLE);
    }
    
    @Override
    public void collectCoral(){
        this.intakeWheels.set(IntakeConstants.tunning_values_intake.INTAKE_SPEED);
        this.indexer.set(IntakeConstants.tunning_values_intake.INDEXER_SPEED);
    }

    @Override
    public void stopIntaking(){
        this.intakeWheels.set(IntakeConstants.tunning_values_intake.STOP_SPEED);
        this.indexer.set(IntakeConstants.tunning_values_intake.STOP_SPEED);
    }

    @Override
    public void goToDefaultPosition(){
        this.intakePivot.setPositionReference(IntakeConstants.tunning_values_intake.setpoints.INTAKE_ANGLE_HOMED);   
    }

    @Override
    public void goToIntakePosition(){
        if(!this.indexerHasCoral){ 
            this.intakePivot.setPositionReference(IntakeConstants.tunning_values_intake.setpoints.INTAKE_ANGLE_COLLECTING); 
        }  
    }      

    @Override
    public void expellCoral(){
        this.intakeWheels.set(IntakeConstants.tunning_values_intake.EXPELL_SPEED);
        this.indexer.set(IntakeConstants.tunning_values_intake.EXPELL_SPEED);
    }

    @Override
    public boolean indexerHasCoral(){
        return indexerHasCoral;
    }

    private void runCoralIntakeDetection(){
        this.indexerHasCoral = this.intakePivot.getLimitSwitch(true);
    }

    @Override
    public void periodic(){
        this.runCoralIntakeDetection();
        this.updateLogs();
    }

    @Override
    public boolean isIntakeAtTargetPosition(double targetPosition){
        return this.intakePivot.getPosition() <= targetPosition + IntakeConstants.tunning_values_intake.ANGLE_ERROR_ALLOWED || 
            this.intakePivot.getPosition() >= targetPosition - IntakeConstants.tunning_values_intake.ANGLE_ERROR_ALLOWED;
    }

    @Override
    public Supplier<Boolean> getIntakeUpSupplier(){
        return () -> this.intakePivot.getPosition() > IntakeConstants.tunning_values_intake.setpoints.INTAKE_ANGLE_FOR_NOT_TOUCHING_PIVOT;
    }


    @Override
    public void setCoastMode(){
        this.intakePivot.setMotorBrake(false);
        this.intakeWheels.setMotorBrake(false);

        this.intakePivot.burnFlash();
        this.intakeWheels.burnFlash();
    }

    @Override
    public void setBrakeMode(){
        this.intakePivot.setMotorBrake(false);
        this.intakeWheels.setMotorBrake(false);

        this.intakePivot.burnFlash();
        this.intakeWheels.burnFlash();
    }

    private void goToTargetPosition(double targetPosition){
        double securedTargetPosition = Math.clamp(targetPosition, IntakeConstants.tunning_values_intake.setpoints.MIN_ANGLE, IntakeConstants.tunning_values_intake.setpoints.MAX_ANGLE);
        this.intakePivot.setPositionReference(securedTargetPosition);
    }
}
