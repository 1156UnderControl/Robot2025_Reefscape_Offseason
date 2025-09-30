// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;

public class MoveScorerToCollectAlgae extends Command {

  private ScorerSubsystem scorer;
  public MoveScorerToCollectAlgae(ScorerSubsystem scorer) {
    this.scorer = scorer;
    addRequirements(scorer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.scorer.moveScorerToCollectAlgae();
    this.scorer.collectAlgae();
    this.scorer.runEndEffectorAlgaeDetection();
  }

  @Override
  public void end(boolean interrupted) {
    this.scorer.holdAlgae();
  }

  @Override
  public boolean isFinished() {
    return this.scorer.hasAlgae();
  }
}
