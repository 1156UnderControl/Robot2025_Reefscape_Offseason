package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;

public class SwerveConstants {
  public static final double DISTANCE_FROM_REEF_CENTER_TO_USE_REEF_POSE_ESTIMATION = 3;
  public static final PIDConfig MOVE_TO_POSE_TRANSLATION_PID = new PIDConfig(5, 0, 0, 0, 0);
  public static final Constraints MOVE_TO_POSE_TRANSLATION_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_X_CONSTRAINTS = new Constraints(4, 3);
  public static final Constraints MOVE_TO_POSE_Y_CONSTRAINTS = new Constraints(4, 3);

  public enum TargetBranch {
    A(1), B(0), C(11), D(10), E(9), F(8), G(7), H(6), I(5), J(4), K(3), L(2);

    private final int branchIndex;

    TargetBranch(int branchIndex) {
      this.branchIndex = branchIndex;
    }

    public Pose2d getTargetPoseToScore() {
      Pose2d transformedPose = CoordinatesTransform.applyRotationToPoseAngle(
          CoordinatesTransform
              .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions2d.get(branchIndex).get(ReefLevel.L4)), 0.6),
          Rotation2d.k180deg);
      return transformedPose;
    }

    public Pose2d getDefaultPoseToScore() {
      Pose2d transformedPose = CoordinatesTransform.applyRotationToPoseAngle(
          CoordinatesTransform
              .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions2d.get(branchIndex).get(ReefLevel.L4)), 0),
          Rotation2d.k180deg);
      return transformedPose;
    }
  }

  public enum TargetFace {
    A(0), B(5), C(4), D(3), E(2), F(1);

    private final int faceIndex;

    TargetFace(int faceIndex) {
      this.faceIndex = faceIndex;
    }

    public Pose2d getTargetPoseToScore() {
      Pose2d transformedPose = CoordinatesTransform.applyRotationToPoseAngle(
          CoordinatesTransform
              .getRetreatPose(AllianceFlipUtil.apply(Reef.centerFaces[faceIndex]), 0.6),
          Rotation2d.k180deg);
      return transformedPose;
    }
  }

  public enum PoseEstimatorState {
    REEF_ESTIMATION, GLOBAL_POSE_ESTIMATION, AUTO_POSE_ESTIMATION;
  }

  public class AutoAlignConstants {
    public class PoseDeadBand {

      public class FastDirect {
        public static final double MAX_ERROR_AUTO_ALIGN_FAST_DIRECT = 1.5;
        public static final double MIN_ERROR_AUTO_ALIGN_FAST_DIRECT = 0.6;
        public static final double ERROR_FOR_ROTATION_ALIGN_ACTIVATION_FAST_DIRECT = 0.3;
      }

      public class Fast {
        public static final double MAX_ERROR_AUTO_ALIGN_FAST = 1.5;
        public static final double MIN_ERROR_AUTO_ALIGN_FAST = 0.7;
        public static final double ERROR_FOR_ROTATION_ALIGN_ACTIVATION_FAST = 0.3;
        public static final double ERROR_FOR_ELEVATOR_RAISED_FAST = 0.6;
      }

      public class RemoveAlgae {
        public static final double MAX_ERROR_AUTO_ALIGN_FAST = 1.5;
        public static final double MIN_ERROR_AUTO_ALIGN_FAST = 0.7;
        public static final double ERROR_FOR_ROTATION_ALIGN_ACTIVATION_FAST = 0.3;
        public static final double ERROR_FOR_ELEVATOR_RAISED_FAST = 0.6;
      }

      public class Autonomous {
        public static final double MAX_ERROR_AUTO_ALIGN_AUTO = 2;
        public static final double MIN_ERROR_AUTO_ALIGN_AUTO = 1.0;
        public static final double ERROR_FOR_ROTATION_ALIGN_ACTIVATION_AUTO = 0.3;
      }

      public class Teleoperated {
        public static final double MAX_ERROR_AUTO_ALIGN_TELEOPERATED = 2.0;
        public static final double MIN_ERROR_AUTO_ALIGN_TELEOPERATED = 1.0;
        public static final double ERROR_FOR_ROTATION_ALIGN_ACTIVATION_TELEOPERATED = 0.3;
      }

      public static final double POSE_TRANSLATION_DEADBAND = 0.04;
      public static final double POSE_HEADING_DEADBAND = 3;
      public static final double POSE_TRANSLATION_DEADBAND_BACKUP = 0.05;
      public static final double POSE_HEADING_DEADBAND_BACKUP = 10;

    }

    public class VelocitiesRelatedToDistance {
      public class FastDirect {
        public static final double MIN_VELOCITY_POSITION = 0.4;
        public static final double MAX_VELOCITY_POSITION = 4;
      }

      public class Fast {
        public static final double MIN_VELOCITY_POSITION = 0.7;
        public static final double MAX_VELOCITY_POSITION = 3.4;
      }

      public class RemoveAlgae {
        public static final double MIN_VELOCITY_POSITION = 1.5;
        public static final double MAX_VELOCITY_POSITION = 3.5;
      }

      public class Autonomous {
        public static final double MIN_VELOCITY_POSITION = 0.7;
        public static final double MID_VELOCITY_POSITION = 1.5;
        public static final double MAX_VELOCITY_POSITION = 3.5;
      }

      public class Teleoperated {
        public static final double MIN_VELOCITY_POSITION = 1.0;
        public static final double MAX_VELOCITY_POSITION = 4.5;
      }

      public static final double MAX_VELOCITY_POSITION = 2;
      public static final double MIN_VELOCITY_POSITION = 1;
      public static final double ELEVATOR_RAISED_VELOCITY_POSITION = 0.7;
    }
  }

  public class CoralStations {
    public class RedAliance {
      public static final Pose2d CORAL_STATION_RIGHT_POSE_FOR_ROBOT = new Pose2d(
          Units.inchesToMeters(33.526),
          Units.inchesToMeters(25.824),
          Rotation2d.fromDegrees(45)).rotateBy(Rotation2d.k180deg);

      public static final Pose2d CORAL_STATION_LEFT_POSE_FOR_ROBOT = new Pose2d(
          Units.inchesToMeters(33.526),
          Units.inchesToMeters(291.176),
          Rotation2d.fromDegrees(-45));
    }

    public class BlueAliance {
      public static final Pose2d CORAL_STATION_RIGHT_POSE_FOR_ROBOT = new Pose2d(
          Units.inchesToMeters(33.526),
          Units.inchesToMeters(25.824),
          Rotation2d.fromDegrees(-135)).rotateBy(Rotation2d.k180deg);

      public static final Pose2d CORAL_STATION_LEFT_POSE_FOR_ROBOT = new Pose2d(
          Units.inchesToMeters(33.526),
          Units.inchesToMeters(291.176),
          Rotation2d.fromDegrees(135));
    }
  }
}
