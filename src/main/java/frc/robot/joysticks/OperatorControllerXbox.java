package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControllerXbox {
  private static OperatorControllerXbox mInstance = null;

  public static OperatorControllerXbox getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorControllerXbox();
    }

    return mInstance;
  }

  CommandPS5Controller controller;

  private OperatorControllerXbox() {
    this.controller = new CommandPS5Controller(1);
  }

  public Trigger goToReefA() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefB() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefC() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefD() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefE() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefF() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefG() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefH() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefI() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefJ() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefK() {
    return new Trigger(() -> false);
  }

  public Trigger goToReefL() {
    return new Trigger(() -> false);
  }

  public Trigger reefL1() {
    return new Trigger(this.controller.cross());
  }

  public Trigger reefL2() {
    return new Trigger(this.controller.circle());
  }

  public Trigger reefL3() {
    return new Trigger(this.controller.square());
  }

  public Trigger reefL4() {
    return new Trigger(this.controller.triangle());
  }

  public Trigger alignToClimb() {
    return new Trigger(() -> false);
  }

  public Trigger climb() {
    return new Trigger(() -> false);
  }

  public Trigger prepareToScore() {
    return new Trigger(() -> false);
  }

  public Trigger scoreObject() {
    return new Trigger(() -> false);
  }

  public Trigger collectCoral() {
    return new Trigger(() -> false);
  }

  public Trigger cancelAction() {
    return new Trigger(() -> false);
  }

  public Trigger removeAlgaeFromBranch() {
    return new Trigger(() -> false);
  }
}
