package frc.Java_Is_UnderControl.Util;

public class JoystickUtil {
  public static boolean withinHypotDeadband(double x, double y) {
    return Math.hypot(x, y) < 0.5;
  }

  public static Boolean inRange(double x, double minX, double maxX) {
    return x > minX && x < maxX;
  }
}
