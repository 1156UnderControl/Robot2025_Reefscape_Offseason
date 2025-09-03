package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkClosedLoopController;

import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Motors.MotorIO.MotorIOInputs;
import frc.Java_Is_UnderControl.Sensors.SensorIO;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class IntakeSubsystem {
    private static IntakeSubsystem instance;

    private final MotorIO intakeRollers;
    private final MotorIO intakePivot;
    private final MotorIO indexer;
    private final SensorIO indexerInfraRed;
    
    private final MotorIOInputsAutoLogged intakeRollersInputs;
    private final MotorIOInputsAutoLogged intakePivotInputs;
    private final MotorIOInputsAutoLogged indexerInputs;

    boolean hasCollected;






    public static IntakeSubsystem getInstance() {
        if (instance == null) {
          instance = new IntakeSubsystem();
        }
        return instance;
    }

    private IntakeSubsystem(){
        this.intakeRollers = new SparkFlexMotor(IntakeConstants.ID_intakeRollersMotor, IntakeConstants.intakeBusID, IntakeConstants.intakeRollersMotorName);
        this.intakePivot = new SparkFlexMotor(IntakeConstants.ID_intakePivotMotor, IntakeConstants.intakeBusID, IntakeConstants.intakePivotMotorName);
        this.indexer = new SparkFlexMotor(IntakeConstants.ID_indexerMotor, IntakeConstants.intakeBusID, IntakeConstants.indexerMotorName);
    }

    private void setConfigsIntakePivot() {
        this.intakePivot.setMotorBrake(true);
        this.intakePivot.setPositionFactor(IntakeConstants.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
        this.intakePivot.setVelocityFactor(IntakeConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND);
        this.intakePivot.configurePIDF(
            IntakeConstants.PID.P,
            IntakeConstants.PID.I,
            IntakeConstants.PID.D,
            0,
            IntakeConstants.PID.IZone);
        this.intakePivot.burnFlash();
        this.intakePivot.setPosition(IntakeConstants.ZERO_POSITION_IN_METERS_FROM_GROUND);
      }
    
    

    
    public void collectCoralIntake(){
    intakeRollers.set(IntakeConstants.tunning_values_intake.DUTY_CYCLE_INTAKE);
}

    public void collectCoralIndexer(){
    indexer.set(IntakeConstants.tunning_values_intake.DUTY_CYCLE_INTAKE);
}

    public void goToIntakePosition(){
        intakePivot.configurePIDF(0.03, 0, 0, 0);
        intakePivot.setPositionReference(1);
    }



}
