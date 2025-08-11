package frc.robot.subsystems.scorer;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.TalonFXMotor;
import frc.Java_Is_UnderControl.Motors.MotorIO.MotorIOInputs;

public class ScorerSubsystem extends SubsystemBase implements ScorerIO{

    private static ScorerSubsystem instance;

    private final MotorIO elevatorLead;
    private final MotorIO elevatorFollower;

    private final ScorerIOInputs scorerInputs;
    private final MotorIOInputs elevatorLeadInputs;
    private final MotorIOInputs elevatorFollowerInputs;

    public static ScorerSubsystem getInstance() {
        if (instance == null) {
          instance = new ScorerSubsystem();
        }
        return instance;
    }

    private ScorerSubsystem(){
        this.scorerInputs = new ScorerIOInputs();
        this.elevatorLeadInputs = new MotorIOInputs();
        this.elevatorFollowerInputs = new MotorIOInputs();

        this.elevatorLead = new TalonFXMotor(0, "can_s0", GravityTypeValue.Elevator_Static, "Elevator Lead Motor");
        this.elevatorFollower = new TalonFXMotor(1, "can_s0", GravityTypeValue.Elevator_Static, "Elevator Follower Motor");
        this.elevatorFollower.setFollower(0, true);
    }

    @Override
    public void setElevatorDutyCicle(double dutyCicle){
        elevatorLead.set(dutyCicle);
    }

    private void updateScorerInputs(ScorerIOInputs scorerInputs) {
        scorerInputs.elevatorLeadPosition = elevatorLeadInputs.position;
        scorerInputs.televatorLeadTargetPosition = elevatorLeadInputs.targetPosition;
        scorerInputs.elevatorLeadVelocity = elevatorLeadInputs.velocity;
        scorerInputs.elevatorLeadTargetVelocity = elevatorLeadInputs.targetSpeed;
        scorerInputs.elevatorLeadIsInverted = elevatorLeadInputs.isInverted;

        scorerInputs.elevatorFollowerPosition = elevatorFollowerInputs.position;
        scorerInputs.televatorFollowerTargetPosition = elevatorFollowerInputs.targetPosition;
        scorerInputs.elevatorFollowerVelocity = elevatorFollowerInputs.velocity;
        scorerInputs.elevatorFollowerTargetVelocity = elevatorFollowerInputs.targetSpeed;
        scorerInputs. elevatorFollowerIsInverted = elevatorFollowerInputs.isInverted;
    }

    @Override
    public void periodic() {
        this.updateScorerInputs(scorerInputs);
        this.elevatorLead.updateInputs(elevatorLeadInputs);
        this.elevatorFollower.updateInputs(elevatorFollowerInputs);
    }
}
