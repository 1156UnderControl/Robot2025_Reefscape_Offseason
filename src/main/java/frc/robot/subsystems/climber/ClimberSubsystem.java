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

    private ClimberSubsystem() {
        this.pivotInputs = new MotorIOInputsAutoLogged();
        this.cageIntakeInputs = new MotorIOInputsAutoLogged();

        this.pivotMotor = new TalonFXMotor(ClimberConstants.ID_PivotMotor, ClimberConstants.CAN_BUS_ID, GravityTypeValue.Arm_Cosine, "Pivot Climber Motor");
        this.cageIntakeMotor = new SparkMAXMotor(ClimberConstants.ID_cageIntakeMotor, ClimberConstants.CAN_BUS_ID, "Cage Intake Motor");

        this.configureClimberMotor();
    }
    
    private void configureClimberMotor() {
        pivotMotor.setInverted(false);
        pivotMotor.setPositionFactor(250);
        pivotMotor.configurePIDF(
            ClimberConstants.PID.P,
            ClimberConstants.PID.I,
            ClimberConstants.PID.D, 0);
        cageIntakeMotor.burnFlash();
        pivotMotor.setMotorBrake(true);
        pivotMotor.setPosition(0);
    }
    
    @Override
    public void periodic(){
        this.updateLogs();
    }

    private void updateLogs() {
        this.pivotMotor.updateInputs(pivotInputs);
        this.cageIntakeMotor.updateInputs(cageIntakeInputs);

        Logger.processInputs("Motors/Climber/Pivot", pivotInputs);
        Logger.processInputs("Motors/Climber/CageIntake", cageIntakeInputs);
    }
    
    @Override
    public void goToClimbedPosition(){
        pivotMotor.setPositionReference(ClimberConstants.setpoints.CLIMBED_ANGLE);
    }

    @Override
    public void goToPrepareClimbPosition(){
        pivotMotor.setPositionReference(ClimberConstants.setpoints.PREPARE_CLIMBED_ANGLE);
    }

    @Override
    public void goToDefaultPosition(){
        pivotMotor.setPositionReference(ClimberConstants.setpoints.DEFAULT_ANGLE);
    }

    @Override
    public boolean isPreparedToIntake(){
        return Math.abs(cageIntakeMotor.getPosition() - ClimberConstants.setpoints.PREPARE_CLIMBED_ANGLE) < 1;
    }

    @Override
    public void setCoastClimber(){
        pivotMotor.setMotorBrake(false);
        cageIntakeMotor.setMotorBrake(false);
    }

    @Override
    public void setBrakeClimber(){
        this.pivotMotor.setMotorBrake(true);
        this.cageIntakeMotor.setMotorBrake(true);
    }

    @Override
    public void startCollectingClimber(){
        this.cageIntakeMotor.set(-1);
    }

    @Override
    public void stopCollectingClimber(){
        this.cageIntakeMotor.set(0);
    }
}
