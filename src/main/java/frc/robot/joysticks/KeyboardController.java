package frc.robot.joysticks;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardController {
  NetworkTable table;

  public KeyboardController() {
    this.table = NetworkTableInstance.getDefault().getTable("/OperatorController");
  }

  private Trigger createTrigger(String key) {
    return new Trigger(() -> this.table.getEntry(key).getBoolean(false));
  }

  public Trigger getATrigger() {
    return createTrigger("a");
  }

  public Trigger getBTrigger() {
    return createTrigger("b");
  }

  public Trigger getCTrigger() {
    return createTrigger("c");
  }

  public Trigger getDTrigger() {
    return createTrigger("d");
  }

  public Trigger getETrigger() {
    return createTrigger("e");
  }

  public Trigger getFTrigger() {
    return createTrigger("f");
  }

  public Trigger getGTrigger() {
    return createTrigger("g");
  }

  public Trigger getHTrigger() {
    return createTrigger("h");
  }

  public Trigger getITrigger() {
    return createTrigger("i");
  }

  public Trigger getJTrigger() {
    return createTrigger("j");
  }

  public Trigger getKTrigger() {
    return createTrigger("k");
  }

  public Trigger getLTrigger() {
    return createTrigger("l");
  }

  public Trigger getMTrigger() {
    return createTrigger("m");
  }

  public Trigger getNTrigger() {
    return createTrigger("n");
  }

  public Trigger getOTrigger() {
    return createTrigger("o");
  }

  public Trigger getPTrigger() {
    return createTrigger("p");
  }

  public Trigger getQTrigger() {
    return createTrigger("q");
  }

  public Trigger getRTrigger() {
    return createTrigger("r");
  }

  public Trigger getSTrigger() {
    return createTrigger("s");
  }

  public Trigger getTTrigger() {
    return createTrigger("t");
  }

  public Trigger getUTrigger() {
    return createTrigger("u");
  }

  public Trigger getVTrigger() {
    return createTrigger("v");
  }

  public Trigger getWTrigger() {
    return createTrigger("w");
  }

  public Trigger getXTrigger() {
    return createTrigger("x");
  }

  public Trigger getYTrigger() {
    return createTrigger("y");
  }

  public Trigger getZTrigger() {
    return createTrigger("z");
  }

  public Trigger get0Trigger() {
    return createTrigger("0");
  }

  public Trigger get1Trigger() {
    return createTrigger("1");
  }

  public Trigger get2Trigger() {
    return createTrigger("2");
  }

  public Trigger get3Trigger() {
    return createTrigger("3");
  }

  public Trigger get4Trigger() {
    return createTrigger("4");
  }

  public Trigger get5Trigger() {
    return createTrigger("5");
  }

  public Trigger get6Trigger() {
    return createTrigger("6");
  }

  public Trigger get7Trigger() {
    return createTrigger("7");
  }

  public Trigger get8Trigger() {
    return createTrigger("8");
  }

  public Trigger get9Trigger() {
    return createTrigger("9");
  }
}
