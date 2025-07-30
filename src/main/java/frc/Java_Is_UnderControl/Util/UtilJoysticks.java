package frc.Java_Is_UnderControl.Util;

public class UtilJoysticks {

  public static boolean withinHypotDeadband(double x, double y) {
    return Math.hypot(x, y) < 0.5;
  }

  public static Boolean inRange(double x, double minX, double maxX) {
    return x > minX && x < maxX;
  }

  public static Boolean atSetpoint(double processVariable, double setpoint, double deadband) {
    return (processVariable > setpoint - deadband) && (processVariable < setpoint + deadband);
  }
}
