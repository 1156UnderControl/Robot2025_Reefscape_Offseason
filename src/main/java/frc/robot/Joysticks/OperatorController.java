package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  KeyboardController keyboard;

  private OperatorController() {
    keyboard = new KeyboardController();
  }

  public Trigger goToReefA() {
    return keyboard.getATrigger();
  }

  public Trigger goToReefB() {
    return keyboard.getBTrigger();
  }

  public Trigger goToReefC() {
    return keyboard.getCTrigger();
  }

  public Trigger goToReefD() {
    return keyboard.getDTrigger();
  }

  public Trigger goToReefE() {
    return keyboard.getETrigger();
  }

  public Trigger goToReefF() {
    return keyboard.getFTrigger();
  }

  public Trigger goToReefG() {
    return keyboard.getGTrigger();
  }

  public Trigger goToReefH() {
    return keyboard.getHTrigger();
  }

  public Trigger goToReefI() {
    return keyboard.getITrigger();
  }

  public Trigger goToReefJ() {
    return keyboard.getJTrigger();
  }

  public Trigger goToReefK() {
    return keyboard.getKTrigger();
  }

  public Trigger goToReefL() {
    return keyboard.getLTrigger();
  }

  public Trigger reefL1() {
    return keyboard.get1Trigger();
  }

  public Trigger reefL2() {
    return keyboard.get2Trigger();
  }

  public Trigger reefL3() {
    return keyboard.get3Trigger();
  }

  public Trigger reefL4() {
    return keyboard.get4Trigger();
  }

  public Trigger alignToClimb() {
    return keyboard.get8Trigger();
  }

  public Trigger climb() {
    return keyboard.get9Trigger();
  }

  public Trigger prepareToScore() {
    return keyboard.get0Trigger();
  }

  public Trigger scoreObject() {
    return keyboard.getPTrigger();
  }

  public Trigger collectCoral() {
    return keyboard.getZTrigger();
  }

  public Trigger cancelAction() {
    return keyboard.getXTrigger();
  }

  public Trigger removeAlgaeFromBranch() {
    return keyboard.getVTrigger();
  }
}
