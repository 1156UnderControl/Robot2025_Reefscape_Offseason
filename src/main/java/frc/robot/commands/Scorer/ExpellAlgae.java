// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scorer.ScorerSubsystem;


public class ExpellAlgae extends Command {
private ScorerSubsystem scorer;
  public ExpellAlgae() {
this.scorer = scorer;
addRequirements(this.scorer);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.scorer.placeAlgae();
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
