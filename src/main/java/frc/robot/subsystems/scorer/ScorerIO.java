package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightScore;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ScorerIO {

    @AutoLog
    public static class ScorerIOInputs{
        public boolean hasCoral = false;
        public boolean hasAlgae = false;
        public boolean isAtPositionPivot = false;
        public boolean isAtPositionElevator = false;
        public ReefLevel targetCoralLevel = ReefLevel.L1;
        public AlgaeHeightReef targetAlgaeLevel = AlgaeHeightReef.MID;
        public AlgaeHeightScore targetAlgaeScoreLevel = AlgaeHeightScore.PROCESSOR;
        public boolean manualScoreCoral = false;
        public boolean manualScoreAlgae = false;
        public boolean pivotSafeMeasuresEnabled = false;
        public String scorerState = "Idle";
        public double minimumHeightElevator = 0.0;
        public double absoluteEncoderPosition = 0.0;
    }

    boolean hasCoral();

    boolean hasAlgae();

    boolean hasObject();

    void collectCoralFromIndexer();

    void runEndEffectorAlgaeDetection();

    void placeCoral();

    void expellAlgae();

    void collectAlgae();

    void setTargetCoralLevel(ReefLevel coralHeightReef);

    void setTargetAlgaeLevel(AlgaeHeightReef algaeHeightReef);

    void setTargetAlgaeLevelToScore(AlgaeHeightScore algaeHeightScore);

    void setAutoAlgaeCollectBranch(TargetBranch autoAlgaeCollectBranch);

    void setManualScoreCoral(boolean manualScoreCoral);
    
    void setManualScoreAlgae(boolean manualScoreAlgae);

    void stopEndEffector();

    void moveToPrepareScoreCoral();

    void moveToScoreCoral();

    void moveScorerToCollectAlgae();

    void moveScorerToScoreAlgae();

    void moveScorerToDefaultPosition();

    void overrideHasCoral();

    void overrideHasAlgae();

    void overrideNoObject();

    boolean isElevatorAtTargetPosition();

    boolean isPivotAtTargetPosition();

    Supplier<ReefLevel> getTargetCoralReefLevelSupplier();

    Supplier<AlgaeHeightReef> getTargetAlgaeReefLevelSupplier();

    void setIntakeUpSupplier(Supplier<Boolean> intakeUpSupplier);

    boolean isElevatorAtTargetPosition(double elevatorTargetPosition);

    boolean isPivotAtTargetPosition(double pivotTargetPosition);

    boolean isScorerAtTargetPosition();

    void setCoastMode();

    void setBrakeMode();

    double getCollectCoralTimer();

    void resetCollectCoralTimer();

    boolean isUpdatingInternalPivotEncoderNecessary();

    void updateInternalPivotEncoder();

    double getEndEffectorAppliedOutput();

    void holdAlgae();

    void holdCoral();
}
