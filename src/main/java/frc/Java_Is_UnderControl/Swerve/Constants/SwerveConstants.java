package frc.Java_Is_UnderControl.Swerve.Constants;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveConstants {
  public static final String FRONT_LEFT_MODULE_NAME = "Front Left Module";
  public static final String FRONT_RIGHT_MODULE_NAME = "Front Right Module";
  public static final String BACK_LEFT_MODULE_NAME = "Back Left Module";
  public static final String BACK_RIGHT_MODULE_NAME = "Back Right Module";

  public static final int MAX_NUMBER_CONFIG_ATTEMPTS = 5;
  public static final double TIMEOUT_SECONDS_CONFIG = 0.25;

  public static final double HIGH_FREQUENCY = 250;
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double LOW_FREQUENCY = 50;

  public static final Lock odometryUpdatesWhileReadingDataStopper = new ReentrantLock();

  public static final double IS_CONNECTED_DEBOUNCE_TIME = 0.5;

  public static final double WHEEL_RADIUS_METERS = 0.04937;
  public static final double ROBOT_MASS = 65.163;
  public static final double ROBOT_MOI = 4.957;
  public static final double WHEEL_COF = 1.430;
  public static final double GEARBOX_REDUCTION = 1.430;
  public static final double ROBOT_SIZE = 0;

  public static final Translation2d[] MODULE_OFFSETS = {
    new Translation2d(10.375, 10.375),
    new Translation2d(10.375, -10.375),
    new Translation2d(-9.75, 9.75),
    new Translation2d(-10.375, -10.375)
  };
}
