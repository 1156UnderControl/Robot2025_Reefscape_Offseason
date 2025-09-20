package frc.Java_Is_UnderControl.Util;

import java.util.concurrent.Callable;

public class StabilizeChecker {
  edu.wpi.first.wpilibj.Timer timer;

  private double stabilityTime;

  public StabilizeChecker() {
    this(0.2);
  }

  public StabilizeChecker(double stabilityTime) {
    this.stabilityTime = stabilityTime;
    this.timer = new edu.wpi.first.wpilibj.Timer();
    timer.reset();
    timer.start();
  }

  public boolean isStableInRange(double value, double setpoint, double range) {
    if (Util.inRange(value, setpoint - range, setpoint + range)) {
      if (this.timer.get() > this.stabilityTime) {
        return true;
      } else {
        return false;
      }
    } else {
      this.timer.reset();
      this.timer.start();
      return false;
    }
  }

  public boolean isStableInCondition(Callable<Boolean> stabilityCondition) {
    boolean isInCondition;
    try {
      isInCondition = stabilityCondition.call();
    } catch (Exception e) {
      isInCondition = false;
    }
    if (isInCondition) {
      if (this.timer.get() > this.stabilityTime) {
        return true;
      } else {
        return false;
      }
    } else {
      this.timer.reset();
      this.timer.start();
      return false;
    }
  }
}
