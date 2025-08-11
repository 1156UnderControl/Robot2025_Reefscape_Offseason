package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ScorerIOInputsAutoLogged extends ScorerIO.ScorerIOInputs implements LoggableInputs, Cloneable{
    @Override
    public void toLog(LogTable table) {
        table.put("/Scorer/Elevator/LeadMotor/Position", elevatorLeadPosition);
        table.put("/Scorer/Elevator/LeadMotor/TargetPosition", elevatorLeadTargetPosition);
        table.put("/Scorer/Elevator/LeadMotor/Velocity", elevatorLeadVelocity);
        table.put("/Scorer/Elevator/LeadMotor/TargetVelocity", elevatorLeadTargetVelocity);
        table.put("/Scorer/Elevator/LeadMotor/IsInverted", elevatorLeadIsInverted);

        table.put("/Scorer/Elevator/FollowerMotor/Position", elevatorFollowerPosition);
        table.put("/Scorer/Elevator/FollowerMotor/TargetPosition", elevatorFollowerTargetPosition);
        table.put("/Scorer/Elevator/FollowerMotor/Velocity", elevatorFollowerVelocity);
        table.put("/Scorer/Elevator/FollowerMotor/TargetVelocity", elevatorFollowerTargetVelocity);
        table.put("/Scorer/Elevator/FollowerMotor/IsInverted", elevatorFollowerIsInverted);
    }

    @Override
    public void fromLog(LogTable table) {
        elevatorLeadPosition = table.get("/Scorer/Elevator/LeadMotor/Position", elevatorLeadPosition);
        elevatorLeadTargetPosition = table.get("/Scorer/Elevator/LeadMotor/TargetPosition", elevatorLeadTargetPosition);
        elevatorLeadVelocity = table.get("/Scorer/Elevator/LeadMotor/Velocity", elevatorLeadVelocity);
        elevatorLeadTargetVelocity = table.get("/Scorer/Elevator/LeadMotor/TargetVelocity", elevatorLeadTargetVelocity);
        elevatorLeadIsInverted = table.get("/Scorer/Elevator/LeadMotor/IsInverted", elevatorLeadIsInverted);

        elevatorFollowerPosition = table.get("/Scorer/Elevator/FollowerMotor/Position", elevatorFollowerPosition);
        elevatorFollowerTargetPosition = table.get("/Scorer/Elevator/FollowerMotor/TargetPosition", elevatorFollowerTargetPosition);
        elevatorFollowerVelocity = table.get("/Scorer/Elevator/FollowerMotor/Velocity", elevatorFollowerVelocity);
        elevatorFollowerTargetVelocity = table.get("/Scorer/Elevator/FollowerMotor/TargetVelocity", elevatorFollowerTargetVelocity);
        elevatorFollowerIsInverted = table.get("/Scorer/Elevator/FollowerMotor/IsInverted", elevatorFollowerIsInverted);
    } 

    @Override
    public ScorerIOInputsAutoLogged clone() {
        ScorerIOInputsAutoLogged copy = new ScorerIOInputsAutoLogged();

        copy.elevatorLeadPosition = this.elevatorLeadPosition;
        copy.elevatorLeadTargetPosition = this.elevatorLeadTargetPosition;
        copy.elevatorLeadVelocity = this.elevatorLeadVelocity;
        copy.elevatorLeadTargetVelocity = this.elevatorLeadTargetVelocity;
        copy.elevatorLeadIsInverted = this.elevatorLeadIsInverted;

        copy.elevatorFollowerPosition = this.elevatorFollowerPosition;
        copy.elevatorFollowerTargetPosition = this.elevatorFollowerTargetPosition;
        copy.elevatorFollowerVelocity = this.elevatorFollowerVelocity;
        copy.elevatorFollowerTargetVelocity = this.elevatorFollowerTargetVelocity;
        copy.elevatorFollowerIsInverted = this.elevatorFollowerIsInverted;

        return copy;
    } 
}  
