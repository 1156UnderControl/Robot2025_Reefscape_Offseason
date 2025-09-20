package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;

public class ScorerSubsystem extends SubsystemBase implements ScorerIO{

    private static ScorerSubsystem instance;

    private final MotorIO elevatorLead;
    private final MotorIO elevatorFollower;

    private final ScorerIOInputsAutoLogged scorerInputs;
    private final MotorIOInputsAutoLogged elevatorLeadInputs;
    private final MotorIOInputsAutoLogged elevatorFollowerInputs;

    public static ScorerSubsystem getInstance() {
        if (instance == null) {
          instance = new ScorerSubsystem();
        }
        return instance;
    }

    private ScorerSubsystem(){
        this.scorerInputs = new ScorerIOInputsAutoLogged();
        this.elevatorLeadInputs = new MotorIOInputsAutoLogged();
        this.elevatorFollowerInputs = new MotorIOInputsAutoLogged();

        this.elevatorLead = new TalonFXMotor(0, "can_s0", GravityTypeValue.Elevator_Static, "Elevator Lead Motor");
        this.elevatorFollower = new TalonFXMotor(1, "can_s0", GravityTypeValue.Elevator_Static, "Elevator Follower Motor");
        this.elevatorFollower.setFollower(0, true);
    }

    @Override
    public void setElevatorDutyCicle(double dutyCicle){
        elevatorLead.set(dutyCicle);
    }

    private void updateScorerInputs(ScorerIOInputsAutoLogged scorerInputs) {
        scorerInputs.elevatorLeadPosition = elevatorLeadInputs.position;
        scorerInputs.elevatorLeadTargetPosition = elevatorLeadInputs.targetPosition;
        scorerInputs.elevatorLeadVelocity = elevatorLeadInputs.velocity;
        scorerInputs.elevatorLeadTargetVelocity = elevatorLeadInputs.targetSpeed;
        scorerInputs.elevatorLeadIsInverted = elevatorLeadInputs.isInverted;

        scorerInputs.elevatorFollowerPosition = elevatorFollowerInputs.position;
        scorerInputs.elevatorFollowerTargetPosition = elevatorFollowerInputs.targetPosition;
        scorerInputs.elevatorFollowerVelocity = elevatorFollowerInputs.velocity;
        scorerInputs.elevatorFollowerTargetVelocity = elevatorFollowerInputs.targetSpeed;
        scorerInputs.elevatorFollowerIsInverted = elevatorFollowerInputs.isInverted;

        Logger.processInputs("Subsystems/Scorer/Elevator/", scorerInputs);
    }

    @Override
    public void periodic() {
        this.updateScorerInputs(scorerInputs);
        this.elevatorLead.updateInputs(elevatorLeadInputs);
        this.elevatorFollower.updateInputs(elevatorFollowerInputs);

        Logger.processInputs("Motors/ElevatorLead/", elevatorLeadInputs);
        Logger.processInputs("Motors/ElevatorFollower/", elevatorFollowerInputs);
    }
}
