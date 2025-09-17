package frc.robot.constants;

public class PivotConstants {
  public static final int pivotBusID = 4;
  public static final int ID_pivotMotor = 15;
  public static final String pivotMotorName = "Pivot Motor";

  public class tunning_values_pivot {
    public static final double MAX_VELOCITY = 750;
    public static final double MAX_ACCELERATION = 10000;
    public static final double ANGLE_FACTOR_ROTOR_ROTATION_TO_MECHANISM_DEGREES = 5.17755415218;
    public static final double ANGLE_FACTOR_MECHANISM_ROTATION_TO_MECHANISM_DEGREES = 540;
    public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND = 9;
    public static final double ZERO_OFFSET_ABSOLUTE_ENCODER = 0.6311826;
    public static final double ANGLE_ERROR_ALLOWED = 1;
    public static final double PIVOT_ANGLE_ERROR_FOR_CONSIDERING_INTAKE_POSITION = 5;

    public class PID {
      public static final double P = 0.025;
      public static final double I = 0.0001;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 2;
    }

    public class setpoints {
      public static final double MAX_ANGLE = 540;
      public static final double MIN_ANGLE = 0;
      public static final double NET_ANGLE = 0;
      public static final double PROCESSOR_ANGLE = 0;
      public static final double L1_ANGLE = 196.5;
      public static final double L2_ANGLE_SCORING = 164.4;
      public static final double L2_ANGLE_PREPARED = 132.3;
      public static final double L3_ANGLE_SCORING = 173.5;
      public static final double L3_ANGLE_PREPARED = 134.2;
      public static final double L4_ANGLE_SCORING = 174.3;
      public static final double L4_ANGLE_PREPARED = 140.6;
      public static final double ALGAE_COLLECT_MID = 0;
      public static final double ALGAE_COLLECT_LOW = 0;
      public static final double ALGAE_COLLECT_GROUND = 0;
      public static final double DEFAULT_ANGLE = 267.5;
      public static final double DEFAULT_ANGLE_WITH_CORAL = 270;
      public static final double DEFAULT_ANGLE_WITH_ALGAE = 90;
      public static final double CORAL_COLLECT_INDEXER = 267.5;
    }
  }
}
