package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Java_Is_UnderControl.Motors.MotorIO;
import frc.Java_Is_UnderControl.Motors.MotorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Motors.NoMotor;
import frc.Java_Is_UnderControl.Motors.SparkFlexMotor;
import frc.Java_Is_UnderControl.Sensors.InfraRed;
import frc.Java_Is_UnderControl.Sensors.SensorIO;
import frc.Java_Is_UnderControl.Sensors.SensorIOInputsAutoLogged;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightScore;
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
    
    private boolean indexerHasCoral;
    private boolean hasCoral;
    private boolean hasAlgae;
    private boolean endEffectorAccelerated = false;
    private ReefLevel coralHeightReef;
    private AlgaeHeightReef algaeHeightReef;
    private AlgaeHeightScore algaeHeightScore;
    private TargetBranch autoAlgaeCollectBranch;

    private String scorerState;

    private double goalElevatorPosition;
    private double goalPivotPosition;

    private boolean manualScoreCoral;
    private boolean manualScoreAlgae;


    private boolean pivotSafeMeasuresEnabled;
    
    private StabilizeChecker stablePositionForAlgaeDetection;

    private Supplier<Boolean> intakeUpSupplier;

    private double securedMinimumTargetElevatorPosition;

    private boolean transitionModeEnabled;

    private Timer collectCoralTimer;

    private boolean goingToTargetPivotPosition;

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
        this.algaeHeightReef = AlgaeHeightReef.LOW;
        this.algaeHeightScore = AlgaeHeightScore.PROCESSOR;
        this.scorerState = "Idle";

        this.goalElevatorPosition = 0;
        this.goalPivotPosition = 0;
        this.manualScoreCoral = false;
        this.manualScoreAlgae = false;
        this.pivotSafeMeasuresEnabled = false;
        this.securedMinimumTargetElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION;
        this.transitionModeEnabled = false;
        this.collectCoralTimer = new Timer();
        this.goingToTargetPivotPosition = false;
        this.stablePositionForAlgaeDetection = new StabilizeChecker(0.2);
    }

    private void updateLogs(ScorerIOInputsAutoLogged scorerInputs) {
        scorerInputs.hasCoral = this.hasCoral;
        scorerInputs.hasAlgae = this.hasAlgae;
        scorerInputs.isAtPositionElevator = isElevatorAtTargetPosition();
        scorerInputs.isAtPositionPivot = isPivotAtTargetPosition();
        scorerInputs.targetCoralLevel = this.coralHeightReef;
        scorerInputs.targetAlgaeLevel = this.algaeHeightReef;
        scorerInputs.targetAlgaeScoreLevel = this.algaeHeightScore;
        scorerInputs.manualScoreCoral = this.manualScoreCoral;
        scorerInputs.manualScoreAlgae = this.manualScoreAlgae;
        scorerInputs.pivotSafeMeasuresEnabled = this.pivotSafeMeasuresEnabled;
        scorerInputs.scorerState = this.scorerState;
        scorerInputs.minimumHeightElevator = this.securedMinimumTargetElevatorPosition;
        scorerInputs.absoluteEncoderPosition = this.pivotMotor.getPositionExternalAbsoluteEncoder();

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
        if(this.isUpdatingInternalPivotEncoderNecessary() && DriverStation.isDisabled()){
            this.updateInternalPivotEncoder();
        }
        this.updateLogs(scorerInputs);
        SmartDashboard.putNumber("CollectCoralTimer", this.getCollectCoralTimer());
    }


    @Override
    public boolean hasCoral() {
        return this.hasCoral;
    }

    @Override
    public boolean hasAlgae() {
        return this.hasAlgae;
    }

    @Override
    public boolean hasObject() {
        return this.hasAlgae || this.hasCoral;
    }

    @Override
    public void collectAlgae(){
        endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE_ALGAE);
    }

    @Override
    public void runEndEffectorAlgaeDetection(){
        if(this.endEffectorMotor.getVelocity() >= EndEffectorConstants.tunning_values_endeffector.MINIMUM_VELOCITY_FOR_DETECTION){
            this.endEffectorAccelerated = true;
        }

        if(this.endEffectorMotor.getVelocity() <= EndEffectorConstants.tunning_values_endeffector.VELOCITY_TO_DETECT_RPM_FALL && endEffectorAccelerated &&
        stablePositionForAlgaeDetection.isStableInCondition(() -> this.isPivotAtTargetPosition() && this.isElevatorAtTargetPosition())){
            this.endEffectorAccelerated = false;
            this.hasAlgae = true;
        }
    }


    @Override
    public void collectCoralFromIndexer(){
        if(!this.hasCoral && !hasAlgae){
            this.transitionModeEnabled = true;
            this.scorerState = "Collecting_Coral_From_Indexer";
            this.setEndEffectorDutyCycle(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_INTAKE_CORAL);
            if(this.endEffectorMotor.getVelocity() > 2000){
                moveElevatorToCollectCoral();
                if(this.isElevatorAtTargetPosition()){
                    this.collectCoralTimer.start();
                }
            }
        }
    }

    @Override
    public boolean isScorerAtTargetPosition(){
        return this.isElevatorAtTargetPosition() && this.isPivotAtTargetPosition();
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
        this.endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_CORAL);
        this.hasCoral = false;
    }

    @Override
    public void expellAlgae() {
        this.setEndEffectorDutyCycle(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_EXPELL_ALGAE);
        this.hasAlgae = false;
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
    public void setTargetAlgaeLevelToScore(AlgaeHeightScore algaeHeightScore){
        this.algaeHeightScore = algaeHeightScore;
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
    public void moveToPrepareScoreCoral(){
        if(hasCoral){
        this.assignmentReefLevelGoalsForPreparing();
        this.setElevatorGoals(this.goalElevatorPosition);
        this.setPivotGoals(this.goalPivotPosition);
        }
    }

    @Override
    public void moveToScoreCoral(){
        this.assignmentReefLevelGoalsForScoring();
        this.endEffectorMotor.set(-0.5);
        this.setElevatorGoals(this.goalElevatorPosition);
        this.setPivotGoals(this.goalPivotPosition);
    }

    @Override
    public void moveScorerToDefaultPosition(){
        this.endEffectorAccelerated = false;
        if(this.hasCoral){
            this.endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_HOLDING_CORAL);
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION_WITH_CORAL;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE_WITH_CORAL;
            this.scorerState = "Default_Position_With_Coral";
        } else if (this.hasAlgae){
            this.endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_HOLDING_ALGAE);
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION_WITH_ALGAE;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE_WITH_ALGAE;
            this.scorerState = "Default_Position_With_Algae";
        } else {
            this.endEffectorMotor.set(0);
            this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.DEFAULT_POSITION;
            this.goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.DEFAULT_ANGLE;
            this.scorerState = "Default_Position_Without_Objects";
        }

        if(this.transitionModeEnabled){
            this.scorerState = "Default_Position_Transitioning";
            if(!this.goingToTargetPivotPosition){
                this.elevatorLead.setPositionReference(ElevatorConstants.tunning_values_elevator.setpoints.SAFE_TO_DEFAULT_POSITION);
                if(this.isElevatorAtTargetPosition(ElevatorConstants.tunning_values_elevator.setpoints.SAFE_TO_DEFAULT_POSITION)){
                    this.setPivotGoals(goalPivotPosition);
                    if(this.isPivotAtTargetPosition()){
                        this.goingToTargetPivotPosition = true;
                    }
                }
            } else {
                this.elevatorLead.setPositionReference(this.goalElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);
                if(this.isElevatorAtTargetPosition()){
                    this.transitionModeEnabled = false;
                    this.goingToTargetPivotPosition = false;
                }
            }
        } else {
            this.setElevatorGoals(this.goalElevatorPosition);
            if(this.isElevatorAtTargetPosition()){
                this.setPivotGoals(goalPivotPosition);
            }
        }
    }

    private void moveElevatorToCollectCoral() {
        this.goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.CORAL_COLLECT_INDEXER;
        this.elevatorLead.setPositionReference(this.goalElevatorPosition);
    }

    @Override
    public void moveScorerToCollectAlgae(){
        this.assignmentReefLevelGoalsForAlgaeCollect();
        if(this.algaeHeightReef == AlgaeHeightReef.GROUND){
            this.setPivotGoals(goalPivotPosition);
            if(this.isPivotAtTargetPosition()){
                this.setElevatorGoals(goalElevatorPosition);
            }
        } else {
            this.setElevatorGoals(goalElevatorPosition);
            if(isElevatorAtTargetPosition()){
                this.setPivotGoals(goalPivotPosition);
            }
        }
    }


    @Override
    public void moveScorerToScoreAlgae(){
        this.assignmentReefLevelGoalsForAlgaeScore();
        this.setPivotGoals(goalPivotPosition);
        if(isPivotAtTargetPosition()){
            this.setElevatorGoals(goalElevatorPosition);
        }
    }

    @Override
    public void stopEndEffector() {
        setEndEffectorDutyCycle(0);
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
    }
  
    @Override
    public void overrideNoObject() {
      this.hasCoral = false;
      this.hasAlgae = false;
    }

    @Override
    public Supplier<ReefLevel> getTargetCoralReefLevelSupplier() {
        return () -> this.coralHeightReef;
    }

    @Override
    public Supplier<AlgaeHeightReef> getTargetAlgaeReefLevelSupplier() {
        return () -> this.algaeHeightReef;
    }

    @Override
    public void setIntakeUpSupplier(Supplier<Boolean> intakeUpSupplier){
        this.intakeUpSupplier = intakeUpSupplier;
    }

    @Override
    public void setCoastMode(){
        this.endEffectorMotor.setMotorBrake(false);
        this.pivotMotor.setMotorBrake(false);
        this.elevatorLead.setMotorBrake(false);
        this.elevatorFollower.setMotorBrake(false);

        this.endEffectorMotor.burnFlash();
        this.pivotMotor.burnFlash();
        this.elevatorLead.burnFlash();
        this.elevatorFollower.burnFlash();
    }

    @Override
    public void setBrakeMode(){
        this.endEffectorMotor.setMotorBrake(true);
        this.pivotMotor.setMotorBrake(true);
        this.elevatorLead.setMotorBrake(true);
        this.elevatorFollower.setMotorBrake(true);

        this.endEffectorMotor.burnFlash();
        this.pivotMotor.burnFlash();
        this.elevatorLead.burnFlash();
        this.elevatorFollower.burnFlash();
    }

    @Override
    public double getCollectCoralTimer(){
        return this.collectCoralTimer.get();
    }

    @Override
    public void resetCollectCoralTimer(){
        this.collectCoralTimer.reset();
    }

    @Override
    public boolean isUpdatingInternalPivotEncoderNecessary(){
        return (Math.abs(this.pivotMotor.getPositionExternalAbsoluteEncoder() - this.pivotMotor.getPosition()) > PivotConstants.tunning_values_pivot.PIVOT_ANGLE_ERROR_FOR_UPDATING_INTERNAL_ENCODER_POSITION);
    }

    @Override
    public void updateInternalPivotEncoder(){
        this.pivotMotor.setPosition(this.pivotMotor.getPositionExternalAbsoluteEncoder());
    }

    @Override
    public double getEndEffectorAppliedOutput(){
        return this.endEffectorMotor.getAppliedOutput();
    }

    @Override
    public void holdAlgae(){
        this.endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_HOLDING_ALGAE);
    }

    @Override
    public void holdCoral(){
        this.endEffectorMotor.set(EndEffectorConstants.tunning_values_endeffector.setpoints.DUTY_CYCLE_HOLDING_CORAL);
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

    private void assignmentReefLevelGoalsForAlgaeCollect(){
        switch (this.algaeHeightReef){
            case GROUND:
            goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.ALGAE_COLLECT_GROUND;
            goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.ALGAE_COLLECT_GROUND;
            break;
        case LOW:
            goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.ALGAE_COLLECT_LOW;
            goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.ALGAE_COLLECT_LOW;
            break;
        case MID:
            goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.ALGAE_COLLECT_MID;
            goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.ALGAE_COLLECT_MID;

            break;
        default:
            break;
    }
}

    private void assignmentReefLevelGoalsForAlgaeScore(){
        switch (this.algaeHeightScore){
        case PROCESSOR:
            goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.PROCESSOR_HEIGHT;
            goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.PROCESSOR_ANGLE;
            break;
        case NET:
            goalElevatorPosition = ElevatorConstants.tunning_values_elevator.setpoints.NET_HEIGHT;
            goalPivotPosition = PivotConstants.tunning_values_pivot.setpoints.NET_ANGLE;
            break;
        default:
            break;
    }
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
        this.elevatorLead.setMinMotorOutput(-0.65);
        this.elevatorLead.setMaxMotorOutput(0.65);
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
        pivotMotor.setMinMotorOutput(-0.65);
        pivotMotor.setMaxMotorOutput(0.65);
        pivotMotor.burnFlash();
        pivotMotor.setPosition(pivotMotor.getPositionExternalAbsoluteEncoder());
    }
    
    private void setConfigsEndEffector() {
        endEffectorMotor.setMotorBrake(false);
        endEffectorMotor.setInverted(false);
        endEffectorMotor.setVelocityFactor(EndEffectorConstants.VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM);
        endEffectorMotor.burnFlash();
    }

    private void setElevatorGoals(double targetElevatorPosition){
        double securedTargetElevatorPosition = Math.clamp(targetElevatorPosition, ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT, ElevatorConstants.tunning_values_elevator.setpoints.MAX_HEIGHT);
        goalElevatorPosition = securedTargetElevatorPosition;
        this.elevatorLead.setPositionReference(securedTargetElevatorPosition, ElevatorConstants.tunning_values_elevator.PID.arbFF);    
    }

    private void setPivotGoals(double targetPivotPosition){
        goalPivotPosition = targetPivotPosition;
        this.pivotMotor.setPositionReference(targetPivotPosition);
    }
}