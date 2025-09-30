package frc.robot.constants;

public class ElevatorConstants {
  public static final int elevatorBusID = 1;
  public static final int ID_elevatorLeaderMotor = 13;
  public static final String elevatorLeaderMotorName = "Elevator Leader Motor";
  public static final int ID_elevatorFollowerMotor = 14;
  public static final String elevatorFollowerMotorName = "Elevator Follower Motor";
  public static final double ELEVATOR_HEIGHT_OFFSET_FROM_GROUND = 0.39624;

  public class tunning_values_elevator {
    public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 0.0412436;
    public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 1;
    public static final double POSITION_ERROR_ALLOWED = 0.05;
    public static final double POSITION_FOR_REDUCING_SWERVE_SPEED = 1.5;

    public class PID {
      public static final double P = 2.45;
      public static final double I = 0.014;
      public static final double D = 0;
      public static final double arbFF = 0.035;
      public static final double IZone = 0.01;
    }

    public class setpoints {
      public static final double MAX_HEIGHT = 1.8;
      public static final double MIN_HEIGHT = 0.39624;
      public static final double NET_HEIGHT = 1.798;
      public static final double PROCESSOR_HEIGHT = 0.54;
      public static final double L1_HEIGHT = 0.85;
      public static final double L2_HEIGHT = 0.61;
      public static final double L3_HEIGHT = 0.98;
      public static final double L4_HEIGHT = 1.68;
      public static final double ALGAE_COLLECT_MID = 1.37;
      public static final double ALGAE_COLLECT_LOW = 0.95;
      public static final double ALGAE_COLLECT_GROUND = 0.757;
      public static final double DEFAULT_POSITION = 0.922;
      public static final double DEFAULT_POSITION_WITH_CORAL = 0.45;
      public static final double DEFAULT_POSITION_WITH_ALGAE = 0.51;
      public static final double SAFE_TO_DEFAULT_POSITION = 1;
      public static final double CORAL_COLLECT_INDEXER = 0.88;
    }

    public class stable_transition {
      public static final double ARM_ANGLE_POINT = 7.520704;
      public static final double ARM_HYPOTENUSE = 0.630709;
      public static final double HIGH_ELEVATOR_SAFETY_MARGIN = 0.2;
      public static final double NORMAL_ELEVATOR_SAFETY_MARGIN = 0.05;
    }
  }
}
