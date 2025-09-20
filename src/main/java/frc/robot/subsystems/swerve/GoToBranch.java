package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.Java_Is_UnderControl.Util.GeomUtil;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;

public class GoToBranch {
  private TargetBranch branch;

  private Pose2d finalTargetPose;

  private double finalVelocity;

  private boolean reefAvoidance = false;

  private String state;

  public double distanceToTargetBranch;
  public double distanceToTargetFace;
  public double distanceToTarget;
  public double deltaVelocity;
  public double deltaDistance;
  public double relationDeltas;
  public double targetVelocity;

  GoToBranchConfiguration configuration;

  public GoToBranch(GoToBranchConfiguration configuration) {
    this.configuration = configuration;
  }

  public void updateBranchData(Pose2d robotPose, Supplier<ReefLevel> scorerTargetReefLevelSupplier,
      Supplier<AlgaeHeightReef> scorerTargetReefLevelAlgaeSupplier,
      Supplier<Boolean> elevatorAtHighPositionSupplier, boolean backup) {

    this.distanceToTargetBranch = branch.getTargetPoseToScore().getTranslation()
        .getDistance(robotPose.getTranslation());
    Pose2d targetBranchScorePose = scorerTargetReefLevelSupplier.get() == ReefLevel.L4
        ? CoordinatesTransform.getRetreatPose(branch.getTargetPoseToScore(), 0.09)
        : branch.getTargetPoseToScore();
    this.distanceToTarget = this
        .getDriveTarget(robotPose, targetBranchScorePose, this.reefAvoidance, backup)
        .getTranslation().getDistance(robotPose.getTranslation());
    finalVelocity = this.calculateRobotMaxVelocity(distanceToTargetBranch, this.configuration.maxVelocity,
        this.configuration.minVelocity,
        this.configuration.maxErrorPose, this.configuration.minErrorPose);
    finalTargetPose = this.getDriveTarget(robotPose, targetBranchScorePose, this.reefAvoidance, backup);
    if (distanceToTargetBranch <= this.configuration.minErrorPose) {
      this.state = "DRIVE_TO_BRANCH_" + this.configuration.goToBranchMode + branch.name() + "_CLOSE";
      return;
    } else if (distanceToTargetBranch <= this.configuration.maxErrorPose) {
      this.state = "DRIVE_TO_BRANCH_" + this.configuration.goToBranchMode + branch.name() + "_FAR";
      return;
    } else {
      this.state = "DRIVE_TO_BRANCH_" + this.configuration.goToBranchMode + branch.name() + "_MID";
      return;
    }
  }

  private Pose2d getDriveTarget(Pose2d robot, Pose2d goal, boolean goDirect, boolean backup) {
    if (backup) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.35, 0.0));
      this.state = "DRIVE_TO_BRANCH_" + this.configuration.goToBranchMode + branch.name() + "BACKUP";
    } else {
      goal = goal.transformBy(GeomUtil.toTransform2d(-0.11, 0.0));
    }
    if (goDirect) {
      return goal;
    } else {
      return this.calculateReefAvoidenceTarget(robot, goal);
    }
  }

  private Pose2d calculateReefAvoidenceTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);// Gets the error
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT = MathUtil.clamp(
        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
        0.0,
        1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * 1.2,
            Math.copySign(shiftYT * 1.5 * 0.8, offset.getY())));
  }

  private double calculateRobotMaxVelocity(double distanceToTarget, double maxVelocity, double minVelocity,
      double maxDistance, double minDistance) {
    this.deltaVelocity = maxVelocity - minVelocity;
    this.deltaDistance = maxDistance - minDistance;
    this.relationDeltas = deltaVelocity / deltaDistance;
    this.targetVelocity = minVelocity + relationDeltas * (distanceToTarget - minDistance);
    if (distanceToTarget > maxDistance) {
      return maxVelocity;
    }
    if (distanceToTarget < minDistance) {
      return minVelocity;
    }
    return targetVelocity;
  }

  public String getGoToState() {
    return this.state;
  }

  public Pose2d getFinalPose() {
    return this.finalTargetPose;
  }

  public double getFinalVelocity() {
    return this.finalVelocity;
  }

  public void setBranch(TargetBranch targetBranch, boolean goDirect) {
    this.branch = targetBranch;
    this.reefAvoidance = goDirect;
  }

  public double getDistanceToTargetBranch() {
    return this.distanceToTargetBranch;
  }

  public double getDistanceToTargetFace() {
    return this.distanceToTargetFace;
  }
}
