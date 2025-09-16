package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.NoMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.Java_Is_UnderControl.Sensors.SensorIO;
import frc.Java_Is_UnderControl.Sensors.SensorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Swerve.constants.SwerveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.constants.PivotConstants;

public class ScorerSubsystem extends SubsystemBase implements ScorerIO{

    private static ScorerSubsystem instance;

    private final MotorIO elevatorLead;
    private final MotorIO elevatorFollower;
    private final MotorIO pivotMotor;
    private final MotorIO endEffectorMotor;
    private final SensorIO EndEffectorInfraRed;

    private final ScorerIOInputsAutoLogged scorerInputs;
    private final MotorIOInputsAutoLogged elevatorLeadInputs;
    private final MotorIOInputsAutoLogged elevatorFollowerInputs;
    private final MotorIOInputsAutoLogged pivotInputs;
    private final MotorIOInputsAutoLogged endEffectorInputs;
    private final SensorIOInputsAutoLogged EndEffectorInfraRedInputs;
    
    private boolean hasCoral;
    private boolean hasAlgae;
    private ReefLevel coralHeightReef;
    private AlgaeHeightReef algaeHeightReef;
    private TargetBranch autoAlgaeCollectBranch;

    private String scorerState;

    private double goalElevatorPosition;
    private double goalPivotPosition;

    private boolean manualScoreCoral;
    private boolean manualScoreAlgae;

    private boolean pivotSafeMeasuresEnabled;

    private boolean goingToIndexerPosition;

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
        this.pivotInputs = new MotorIOInputsAutoLogged();
        this.endEffectorInputs = new MotorIOInputsAutoLogged();
        this.EndEffectorInfraRedInputs = new SensorIOInputsAutoLogged();

        this.elevatorLead = new SparkFlexMotor(ElevatorConstants.ID_elevatorLeaderMotor, ElevatorConstants.elevatorBusID, ElevatorConstants.elevatorLeaderMotorName);
        this.elevatorFollower = new SparkFlexMotor(ElevatorConstants.ID_elevatorFollowerMotor, ElevatorConstants.elevatorBusID, ElevatorConstants.elevatorFollowerMotorName);
        this.pivotMotor = new SparkFlexMotor(PivotConstants.ID_pivotMotor, PivotConstants.pivotBusID, PivotConstants.pivotMotorName);
        this.endEffectorMotor = new SparkFlexMotor(EndEffectorConstants.ID_endEffectorMotor, EndEffectorConstants.endEffectorBusID, EndEffectorConstants.endEffectorMotorName);
        this.EndEffectorInfraRed = new InfraRed(EndEffectorConstants.Port_coralInfraRed, false);

        this.setConfigsElevator();
        this.setConfigsPivot();
        this.setConfigsEndEffector();

        this.hasCoral = DriverStation.isAutonomous();
        this.hasAlgae = false;
        this.coralHeightReef = ReefLevel.L1;
        this.scorerState = "Idle";

        this.goalElevatorPosition = 0;
        this.goalPivotPosition = 0;
        this.manualScoreCoral = false;
        this.manualScoreAlgae = false;
        this.pivotSafeMeasuresEnabled = false;
        this.goingToIndexerPosition = false;
    }

    private void updateLogs(ScorerIOInputsAutoLogged scorerInputs) {
        scorerInputs.hasCoral = this.hasCoral;
        scorerInputs.hasAlgae = this.hasAlgae;
        scorerInputs.targetCoralLevel = this.coralHeightReef;
        scorerInputs.targetAlgaeLevel = this.algaeHeightReef;
        scorerInputs.manualScoreCoral = this.manualScoreCoral;
        scorerInputs.manualScoreAlgae = this.manualScoreAlgae;
        scorerInputs.pivotSafeMeasuresEnabled = this.pivotSafeMeasuresEnabled;
        scorerInputs.scorerState = this.scorerState;

        this.elevatorLead.updateInputs(elevatorLeadInputs);
        this.elevatorFollower.updateInputs(elevatorFollowerInputs);
        this.pivotMotor.updateInputs(pivotInputs);
        this.endEffectorMotor.updateInputs(endEffectorInputs);
        this.EndEffectorInfraRed.updateInputs(EndEffectorInfraRedInputs);

        Logger.processInputs("Motors/Scorer/ElevatorLead/", elevatorLeadInputs);
        Logger.processInputs("Motors/Scorer/ElevatorFollower/", elevatorFollowerInputs);
        Logger.processInputs("Motors/Scorer/Pivot/", pivotInputs);
        Logger.processInputs("Motors/Scorer/EndEffector/", endEffectorInputs);
        Logger.processInputs("Subsystems/Scorer/", scorerInputs);
        Logger.processInputs("Sensors/Scorer/EndEffectorInfraRed/", EndEffectorInfraRedInputs);
    }

    @Override
    public void periodic() {
        this.updateLogs(scorerInputs);
    }

    @Override
    public boolean hasCoral() {
        return EndEffectorInfraRed.getBoolean();
    }

    @Override
    public boolean hasAlgae() {
        return this.hasAlgae;
    }

    @Override
    public void collectCoralFromIndexer(){
        if(!this.hasCoral && !hasAlgae){
            this.scorerState = "Collecting_Coral_From_Indexer";
            if(this.isEndEffectorAtTargetVelocity(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE_CORAL)){
                this.setScorerStructureGoals(ElevatorConstants.tunning_values_elevator.setpoints.CORAL_COLLECT_INDEXER, PivotConstants.tunning_values_pivot.setpoints.CORAL_COLLECT_INDEXER);
            } else {
                this.setEndEffectorDutyCycle(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE_CORAL);
            }
        }
    }

    @Override
    public boolean isElevatorAtTargetPosition() {
        return this.isElevatorAtTargetPosition(this.goalElevatorPosition);
    }

    @Override
    public boolean isPivotAtTargetPosition() {
        return this.isPivotAtTargetPosition(this.goalPivotPosition);
    }

    @Override
    public boolean isElevatorAtTargetPosition(double elevatorTargetPosition) {
        return this.elevatorLead.getPosition() <= elevatorTargetPosition + ElevatorConstants.tunning_values_elevator.POSITION_ERROR_ALLOWED &&
        this.elevatorLead.getPosition() >= elevatorTargetPosition - ElevatorConstants.tunning_values_elevator.POSITION_ERROR_ALLOWED;
    }

    @Override
    public boolean isPivotAtTargetPosition(double pivotTargetPosition) {
        return this.pivotMotor.getPosition() <= pivotTargetPosition + PivotConstants.tunning_values_pivot.ANGLE_ERROR_ALLOWED &&
        this.pivotMotor.getPosition() >= pivotTargetPosition - PivotConstants.tunning_values_pivot.ANGLE_ERROR_ALLOWED;
    }

    private boolean isEndEffectorAtTargetVelocity(double targetAppliedOutput){
        return this.endEffectorMotor.getAppliedOutput() <= targetAppliedOutput + EndEffectorConstants.tunning_values_endeffector.APPLIED_OUTPUT_ERROR_ALLOWED &&
        this.pivotMotor.getPosition() >= targetAppliedOutput - EndEffectorConstants.tunning_values_endeffector.APPLIED_OUTPUT_ERROR_ALLOWED;
    }

    @Override
    public void placeCoral() {
        if(this.hasCoral){
            if(this.isEndEffectorAtTargetVelocity(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_CORAL)){

            } else {
                this.setEndEffectorDutyCycle(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_CORAL);
            }
        }
    }

    @Override
    public void placeAlgae() {
        if(this.hasAlgae){
            this.setEndEffectorDutyCycle(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_ALGAE);
        }
    }

    @Override
    public void setTargetCoralLevel(ReefLevel coralHeightReef) {
        this.coralHeightReef = coralHeightReef;
    }

    @Override
    public void setTargetAlgaeLevel(AlgaeHeightReef algaeHeightReef) {
        this.algaeHeightReef = algaeHeightReef;
    }

    @Override
    public void setAutoAlgaeCollectBranch(TargetBranch autoAlgaeCollectBranch) {
        this.autoAlgaeCollectBranch = autoAlgaeCollectBranch;
    }

    @Override
    public void setManualScoreCoral(boolean manualScoreCoral){
        this.manualScoreCoral = manualScoreCoral;
    }

    @Override
    public void setManualScoreAlgae(boolean manualScoreAlgae){
        this.manualScoreAlgae = manualScoreAlgae;
    }

    @Override
    public void prepareToScoreAlgae(){
        this.setScorerTargetAlgae();
        this.setScorerStructureGoals(goalElevatorPosition, goalPivotPosition);
    }

    @Override
    public void prepareToScoreCoral(){
        this.assignmentReefLevelGoalsForPreparing();
        this.setScorerStructureGoals(this.goalElevatorPosition, this.goalPivotPosition);
    }

    @Override
    public void scoreCoral(){
        this.assignmentReefLevelGoalsForScoring();
        this.setScorerStructureGoals(this.goalElevatorPosition, this.goalPivotPosition);
    }

    @Override
    public void moveScorerToDefaultPosition(){
        if(this.hasCoral){
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION_WITH_CORAL;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE_WITH_CORAL;
        } else if (this.hasAlgae){
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION_WITH_ALGAE;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE_WITH_ALGAE;
        } else {
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
            this.scorerState = "Default_Position_Without_Objects";
        }

        this.setScorerStructureGoals(goalElevatorPosition, goalPivotPosition);
    }

    private void setEndEffectorDutyCycle(double dutyCycle){
        this.endEffectorMotor.set(dutyCycle);
    }

    @Override
    public void overrideHasCoral() {
      this.hasCoral = true;
      this.hasAlgae = false;
    }
  
    @Override
    public void overrideHasAlgae() {
      this.hasAlgae = true;
      this.hasCoral = false;
    }
  
    @Override
    public void overrideNoObject() {
      this.hasCoral = false;
      this.hasAlgae = false;
    }

    @Override
    public Supplier<Boolean> getReefScoringModeSupplier() {
        return () -> this.elevatorLead.getPosition() > ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
    }

    @Override
    public Supplier<ReefLevel> getTargetCoralReefLevelSupplier() {
        return () -> this.coralHeightReef;
    }

    @Override
    public Supplier<AlgaeHeightReef> getTargetAlgaeReefLevelSupplier() {
        return () -> this.algaeHeightReef;
    }

    private void assignmentReefLevelGoalsForPreparing(){
        switch (this.coralHeightReef) {
            case L1:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L1_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L1_ANGLE;
                break;
            case L2:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L2_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L2_ANGLE_PREPARED;
                break;
            case L3:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L3_ANGLE_PREPARED;
                break;
            case L4:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L4_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L4_ANGLE_PREPARED;
                break;
            default:
                break;
        }
    }

    private void assignmentReefLevelGoalsForScoring(){
        switch (this.coralHeightReef) {
            case L1:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L1_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L1_ANGLE;
                break;
            case L2:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L2_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L2_ANGLE_SCORING;
                break;
            case L3:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L3_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L3_ANGLE_SCORING;
                break;
            case L4:
                goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.L4_HEIGHT;
                goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.L4_ANGLE_SCORING;
                break;
            default:
                break;
        }
    }

    private void setScorerTargetAlgae(){
        
    }

    private void setConfigsElevator() {
        this.elevatorLead.setInverted(true);
        this.elevatorLead.setMotorBrake(true);
        this.elevatorFollower.setMotorBrake(true);
        this.elevatorFollower.setFollower(ElevatorConstants.ID_elevatorLeaderMotor, true);
        this.elevatorLead.setPositionFactor(ElevatorConstants.tunning_values_elevator.POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS);
        this.elevatorLead.setVelocityFactor(ElevatorConstants.tunning_values_elevator.VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND);
        this.elevatorLead.configurePIDF(
            ElevatorConstants.tunning_values_elevator.PID.P,
            ElevatorConstants.tunning_values_elevator.PID.I,
            ElevatorConstants.tunning_values_elevator.PID.D,
            ElevatorConstants.tunning_values_elevator.PID.arbFF,
            ElevatorConstants.tunning_values_elevator.PID.IZone);
        this.elevatorFollower.burnFlash();
        this.elevatorLead.burnFlash();
        this.elevatorLead.setPosition(ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_FROM_GROUND);
      }
    
    private void setConfigsPivot() {
        pivotMotor.setInverted(true);
        pivotMotor.setInvertedEncoder(true);
        pivotMotor.setMotorBrake(true);
        pivotMotor.setPositionFactor(PivotConstants.tunning_values_pivot.ANGLE_FACTOR_ROTOR_ROTATION_TO_MECHANISM_DEGREES);
        pivotMotor.setPositionFactorExternalEncoder(PivotConstants.tunning_values_pivot.ANGLE_FACTOR_MECHANISM_ROTATION_TO_MECHANISM_DEGREES);
        pivotMotor.setVelocityFactorExternalEncoder(PivotConstants.tunning_values_pivot.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND);
        pivotMotor.setAbsoluteEncoderZeroOffset(PivotConstants.tunning_values_pivot.ZERO_OFFSET_ABSOLUTE_ENCODER);
        pivotMotor.configurePIDF(
            PivotConstants.tunning_values_pivot.PID.P,
            PivotConstants.tunning_values_pivot.PID.I,
            PivotConstants.tunning_values_pivot.PID.D,
            PivotConstants.tunning_values_pivot.PID.arbFF,
            PivotConstants.tunning_values_pivot.PID.IZone);
        pivotMotor.setMinMotorOutput(-0.8);
        pivotMotor.setMaxMotorOutput(0.8);
        pivotMotor.setPosition(pivotMotor.getPositionExternalAbsoluteEncoder());
        pivotMotor.burnFlash();
    }
    
    private void setConfigsEndEffector() {
        endEffectorMotor.setMotorBrake(false);
        endEffectorMotor.setInverted(false);
        endEffectorMotor.setVelocityFactor(EndEffectorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM);
        endEffectorMotor.burnFlash();
    }

    private void setScorerStructureGoals(double targetElevatorPosition, double targetPivotPosition){ 
        double securedMinimumTargetElevatorPosition;
        double securedTargetElevatorPosition;

        securedTargetElevatorPosition = Math.clamp(targetElevatorPosition, ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT, ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT);

        if((this.pivotMotor.getPosition() <= 270 && targetPivotPosition <= 270) || (this.pivotMotor.getPosition() > 270 && targetPivotPosition > 270)){
            securedMinimumTargetElevatorPosition = this.minimumHeightElevator(Math.abs(((targetPivotPosition % 360) - 270) - ElevatorConstants.tunning_values_elevator.stable_transition.ARM_ANGLE_POINT));
            if(securedTargetElevatorPosition == ElevatorConstants.tunning_values_elevator.setpoints.CORAL_COLLECT_INDEXER){
                if((this.elevatorLead.getPosition() > securedMinimumTargetElevatorPosition || this.isElevatorAtTargetPosition(securedMinimumTargetElevatorPosition)) || this.goingToIndexerPosition){
                    this.pivotSafeMeasuresEnabled = false;
                    if(this.isPivotAtTargetPosition(targetPivotPosition)){
                        this.goingToIndexerPosition = true;
                        this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    } else {
                        this.pivotMotor.setPositionReference(targetPivotPosition);
                    }
                } else {
                    this.pivotSafeMeasuresEnabled = true;
                    this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition);
                }
                return;
            }
            if(this.elevatorLead.getPosition() > securedMinimumTargetElevatorPosition || this.isElevatorAtTargetPosition(securedMinimumTargetElevatorPosition)){
                this.pivotSafeMeasuresEnabled = false;
                if(securedTargetElevatorPosition >= securedMinimumTargetElevatorPosition){
                    this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    this.pivotMotor.setPositionReference(targetPivotPosition);
                } else {
                    if(this.isPivotAtTargetPosition(targetPivotPosition)){
                        this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    } else {
                        this.pivotMotor.setPositionReference(targetPivotPosition);
                    }
                }
            } else {
                this.pivotSafeMeasuresEnabled = true;
                if(securedTargetElevatorPosition < securedMinimumTargetElevatorPosition){
                    this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                } else {
                    this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                }
            }

        } else {
            securedMinimumTargetElevatorPosition = this.minimumHeightElevator(270);
            if(securedTargetElevatorPosition == ElevatorConstants.tunning_values_elevator.setpoints.CORAL_COLLECT_INDEXER){
                if(this.elevatorLead.getPosition() > securedMinimumTargetElevatorPosition || this.isElevatorAtTargetPosition(securedMinimumTargetElevatorPosition)){
                    this.pivotSafeMeasuresEnabled = false;
                    if(this.isPivotAtTargetPosition(targetPivotPosition)){
                        this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    } else {
                        this.pivotMotor.setPositionReference(targetPivotPosition);
                    }
                } else {
                    this.pivotSafeMeasuresEnabled = true;
                    this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition);
                }
            }
            if(this.elevatorLead.getPosition() > securedMinimumTargetElevatorPosition || this.isElevatorAtTargetPosition(securedMinimumTargetElevatorPosition)){
                this.pivotSafeMeasuresEnabled = false;
                if(this.isPivotAtTargetPosition(targetPivotPosition)){
                    if(securedTargetElevatorPosition < this.minimumHeightElevator(this.pivotMotor.getPosition())){
                        this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    } else {
                        this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                    }
                } else {
                    this.pivotMotor.setPositionReference(targetPivotPosition);
                }
            } else {
                this.pivotSafeMeasuresEnabled = true;
                if(securedTargetElevatorPosition < this.minimumHeightElevator(this.pivotMotor.getPosition())){
                    this.elevatorLead.setPositionReference(securedMinimumTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                } else {
                    this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                }
            }
        }
    }

    private double minimumHeightElevator(double alphaPivotAngle) {
        double angleRad = Math.toRadians(alphaPivotAngle);
        double angleDeg = (alphaPivotAngle % 360 + 360) % 360;

        if (angleDeg == 270) {
            return ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION;
        }

        if (angleDeg % 90 == 0) {
            return ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT;
        }
    
        if (Math.sin(angleRad) * ElevatorConstants.tunning_values_elevator.stable_transition.ARM_HYPOTENUSE 
                <= SwerveConstants.ROBOT_SIZE / 2) {
            return Math.cos(angleRad) * ElevatorConstants.tunning_values_elevator.stable_transition.ARM_HYPOTENUSE
                   + (ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_FROM_GROUND);
        }
    
        return ( (SwerveConstants.ROBOT_SIZE / 2) / Math.tan(angleRad) ) 
               + (ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_FROM_GROUND);
    }
}