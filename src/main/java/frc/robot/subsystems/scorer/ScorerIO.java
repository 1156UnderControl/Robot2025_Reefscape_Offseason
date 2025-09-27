package frc.robot.subsystems.scorer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
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

    boolean runEndEffectorAlgaeDetection();

    void placeCoral();

    void placeAlgae();

    void collectAlgae();

    void setTargetCoralLevel(ReefLevel coralHeightReef);

    void setTargetAlgaeLevel(AlgaeHeightReef algaeHeightReef);

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

    Supplier<Boolean> getReefScoringModeSupplier();

    Supplier<ReefLevel> getTargetCoralReefLevelSupplier();

    Supplier<AlgaeHeightReef> getTargetAlgaeReefLevelSupplier();

    void setIntakeUpSupplier(Supplier<Boolean> intakeUpSupplier);

    boolean isElevatorAtTargetPosition(double elevatorTargetPosition);

    boolean isPivotAtTargetPosition(double pivotTargetPosition);

    boolean isScorerAtTargetPosition();

    void setCoastMode();

    void setBrakeMode();

    double getCollectTimer();

    void resetCollectTimer();

    boolean isUpdatingInternalPivotEncoderNecessary();

    void updateInternalPivotEncoder();

    double getEndEffectorAppliedOutput();
}
