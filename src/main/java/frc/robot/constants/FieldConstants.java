// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428),
        Units.inchesToMeters(286.779));
    public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428),
        Units.inchesToMeters(242.855));
    public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428),
        Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace = new Pose2d(
        Units.inchesToMeters(33.526),
        Units.inchesToMeters(291.176),
        Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace = new Pose2d(
        Units.inchesToMeters(33.526),
        Units.inchesToMeters(25.824),
        Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
        Units.inchesToMeters(158.501));
    public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef
                                                                          // zone line

    public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions = new ArrayList<>(20); // Starting at the right
    // branch facing the driver
    // station in clockwise

    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    static {
      // Initialize faces
      var aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          // double adjustY = Units.inchesToMeters(6.469);
          double adjustY = Units.inchesToMeters(7.5);

          var rightBranchPose = new Pose3d(
              new Translation3d(
                  poseDirection
                      .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                      .getY(),
                  level.height),
              new Rotation3d(
                  0,
                  Units.degreesToRadians(level.pitch),
                  poseDirection.getRotation().getRadians()));
          var leftBranchPose = new Pose3d(
              new Translation3d(
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                      .getX(),
                  poseDirection
                      .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                      .getY(),
                  level.height),
              new Rotation3d(
                  0,
                  Units.degreesToRadians(level.pitch),
                  poseDirection.getRotation().getRadians()));

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
        }
        branchPositions.add((face * 2), fillRight);
        branchPositions.add((face * 2) + 1, fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5),
        new Rotation2d());
    public static final Pose2d middleIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5),
        new Rotation2d());
    public static final Pose2d rightIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5),
        new Rotation2d());
  }

  public enum ReefLevel {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    TO_L4(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  public class Algae {
    public enum AlgaeHeightReef {
      MID(Units.inchesToMeters(72)),
      LOW(Units.inchesToMeters(47.625)),
      GROUND(Units.inchesToMeters(ElevatorConstants.tunning_values_elevator.setpoints.MIN_HEIGHT));

      AlgaeHeightReef(double height) {
        this.height = height;
      }

      public final double height;
    }

    public enum AlgaeHeightScore {
      NET(Units.inchesToMeters(72)),
      PROCESSOR(Units.inchesToMeters(47.625));

      AlgaeHeightScore(double height) {
        this.height = height;
      }

      public final double height;
    }
  }

}
