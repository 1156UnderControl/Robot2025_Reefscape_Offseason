package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;

public interface ScorerIO {

    @AutoLog
    public static class ScorerIOInputs{
        public boolean hasCoral = false;
        public boolean hasAlgae = false;
        public ReefLevel targetLevel = ReefLevel.L1;
        public String scorerState = "Idle";
    }

    boolean hasCoral();

    boolean hasAlgae();

    void collectCoralFromIndexer();

    void placeCoral();

    void placeAlgae();

    void setTargetCoralLevel(ReefLevel coralHeightReef);

    void setTargetAlgaeLevel(AlgaeHeightReef algaeHeightReef);

    void setAutoAlgaeCollectBranch(TargetBranch autoAlgaeCollectBranch);

    void setManualScoreCoral(boolean manualScoreCoral);
    
    void setManualScoreAlgae(boolean manualScoreAlgae);

    void prepareToScoreAlgae();

    void prepareToScoreCoral();

    void moveScorerToDefaultPosition();

    void overrideHasCoral();

    void overrideHasAlgae();

    void overrideNoObject();

    boolean isElevatorAtTargetPosition();

    boolean isPivotAtTargetPosition();
}
