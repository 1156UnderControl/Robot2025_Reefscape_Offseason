package frc.robot.constants;

public class ElevatorConstants {
  public static final int elevatorBusID = 0;
  public static final int ID_elevatorLeaderMotor = 0;
  public static final String elevatorLeaderMotorName = "Elevator Leader Motor";
  public static final int ID_elevatorFollowerMotor = 1;
  public static final String elevatorFollowerMotorName = "Elevator Follower Motor";
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 0;
  public static final double ZERO_POSITION_IN_METERS_FROM_GROUND = 0;
  public static final double PASSIVE_HOMING_RANGE = 0;

  public class tunning_values_elevator {
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double POSITION_ERROR_ALLOWED = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 0;
    }

    public class setpoints {
      public static final double MAX_HEIGHT = 0;
      public static final double MIN_HEIGHT = 0;
      public static final double NET_HEIGHT = 0;
      public static final double PROCESSOR_HEIGHT = 0;
      public static final double L1_HEIGHT = 0;
      public static final double L2_HEIGHT = 0;
      public static final double L3_HEIGHT = 0;
      public static final double L4_HEIGHT = 0;
      public static final double ALGAE_COLLECT_MID = 0;
      public static final double ALGAE_COLLECT_LOW = 0;
      public static final double ALGAE_COLLECT_GROUND = 0;
      public static final double DEFAULT_POSITION = 0;
      public static final double DEFAULT_POSITION_WITH_CORAL = 0;
      public static final double DEFAULT_POSITION_WITH_ALGAE = 0;
      public static final double PIVOT_SAFE_FOR_ELEVATOR = 0;
    }

    public class stable_transition {
      public static final double DISTANCE_FOR_FULL_DEPLOYMENT = 0;
      public static final double DISTANCE_FOR_DEPLOYMENT_START = 0;
      public static final double SAFE_CRUISE_HEIGHT = 0;
    }
  }
}
