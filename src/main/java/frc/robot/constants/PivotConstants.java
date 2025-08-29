package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {
  public static final int pivotBusID = 1;
  public static final int ID_pivotMotor = 2;
  public static final String pivotMotorName = "Pivot Motor";
  public static final double ANGLE_FACTOR_MECHANISM_ROTATION_TO_MECHANISM_DEGREES = 0;
  public static final double ANGLE_FACTOR_ROTOR_ROTATION_TO_MECHANISM_DEGREES = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND = 0;
  public static final double ZERO_OFFSET_ABSOLUTE_ENCODER = 0;
  public static final double ARM_LENGH = 0;
  public static final double ARM_WIDHT = 0;
  public static final double ARM_ANGLE = 0;
  public static final Pose2d BREAK_POINT_POSE_PIVOT = new Pose2d(0, 0.1, new Rotation2d());

  public class tunning_values_pivot {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_VELOCITY_WITH_ALGAE = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double ANGLE_ERROR_ALLOWED = 0;
    public static final double MIN_DEAD_BAND_FOR_MOTOR_STOP = 0;
    public static final double MAX_DEAD_BAND_FOR_MOTOR_STOP = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 0;
    }

    public class setpoints {
      public static final double MAX_ANGLE = 540;
      public static final double MIN_ANGLE = 0;
      public static final double NET_ANGLE = 0;
      public static final double PROCESSOR_ANGLE = 0;
      public static final double L1_ANGLE = 0;
      public static final double L2_ANGLE = 0;
      public static final double L3_ANGLE = 0;
      public static final double L4_ANGLE = 0;
      public static final double ALGAE_COLLECT_MID = 0;
      public static final double ALGAE_COLLECT_LOW = 0;
      public static final double ALGAE_COLLECT_GROUND = 0;
      public static final double DEFAULT_ANGLE = 0;
      public static final double DEFAULT_ANGLE_WITH_CORAL = 0;
      public static final double DEFAULT_ANGLE_WITH_ALGAE = 0;
    }
  }
}
