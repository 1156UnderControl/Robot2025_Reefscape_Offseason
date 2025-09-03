package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.MotorIO.MotorIOInputs;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.Java_Is_UnderControl.Sensors.SensorIO;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class IntakeSubsystem extends SubsystemBase implements IntakeIO{
    private static IntakeSubsystem instance;

    private final MotorIO intakeWheels;
    private final MotorIO intakePivot;
    private final MotorIO indexer;
    private final DigitalInput indexerInfraRed;
    
    private final MotorIOInputsAutoLogged intakeWheelsInputs;
    private final MotorIOInputsAutoLogged intakePivotInputs;
    private final MotorIOInputsAutoLogged indexerInputs;

    private boolean hasCollected;
    private boolean isIntakePivotAtTargetPosition;

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
        this.indexerInfraRed = new DigitalInput(IntakeConstants.port_IR);

        this.intakeWheelsInputs = new MotorIOInputsAutoLogged();
        this.intakePivotInputs = new MotorIOInputsAutoLogged();
        this.indexerInputs = new MotorIOInputsAutoLogged();

        this.setConfigsIntakePivot();
    }

    private void updateLogs(){
        this.intakeWheels.updateInputs(intakeWheelsInputs); 
        this.intakePivot.updateInputs(intakePivotInputs);
        this.indexer.updateInputs(indexerInputs);

        Logger.processInputs("Motors/Intake/Wheels", intakeWheelsInputs);
        Logger.processInputs("Motors/Intake/Pivot", intakePivotInputs);
        Logger.processInputs("Motors/Intake/Indexer", indexerInputs);
    }

    private void setConfigsIntakePivot() {
        this.intakePivot.setMotorBrake(true);
        this.intakePivot.setPositionFactor(IntakeConstants.setpoints.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_ANGLE);
        this.intakePivot.setVelocityFactor(IntakeConstants.setpoints.VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND);
        this.intakePivot.configurePIDF(
            IntakeConstants.PID.P,
            IntakeConstants.PID.I,
            IntakeConstants.PID.D,
            0,
            IntakeConstants.PID.IZone);
        this.intakePivot.burnFlash();
        this.intakePivot.setPosition(IntakeConstants.setpoints.ZERO_POSITION_IN_METERS_FROM_GROUND);
    }
    
    @Override
    public void collectCoral(){
    this.intakeWheels.set(IntakeConstants.tunning_values_intake.INTAKE_SPEED);
    this.indexer.set(IntakeConstants.tunning_values_intake.INDEXER_SPEED);
    }

    public void goToIntakePosition(){
        this.intakePivot.setPositionReference(IntakeConstants.setpoints.INTAKE_ANGLE_COLLECTING);   
    }

    public void goToDefaultPosition(){
        this.intakePivot.setPositionReference(IntakeConstants.setpoints.INTAKE_ANGLE_HOMED);
    }
    
    public void stopIntaking(){
        this.intakeWheels.set(IntakeConstants.tunning_values_intake.STOP_SPEED);
        this.indexer.set(IntakeConstants.tunning_values_intake.STOP_SPEED);
    }

    public void expellCoral(){
        this.intakeWheels.set(IntakeConstants.tunning_values_intake.EXPELL_SPEED);
        this.indexer.set(IntakeConstants.tunning_values_intake.EXPELL_SPEED);
    }


    public boolean indexerHasCoral(){
        return indexerInfraRed.get();
    }

    @Override
    public void periodic(){
        this.updateLogs();
    }
}
