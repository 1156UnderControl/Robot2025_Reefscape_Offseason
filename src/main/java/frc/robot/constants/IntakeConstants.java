package frc.robot.constants;

public class IntakeConstants {
  public static final int ID_intakeRollersMotor = 0;
  public static final int ID_intakePivotMotor = 0;
  public static final int ID_indexerMotor = 0;
  public static final int intakeBusID = 0;
  public static final String intakeRollersMotorName = "Intake Rollers";
  public static final String intakePivotMotorName = "Intake Pivot";
  public static final String indexerMotorName = "Indexer";
  public static final int port_IR = 0;
  public static final Pose2d BREAK_POINT_POSE_PIVOT_CLOSED = new Pose2d(0.517806, 0.330912, new Rotation2d());
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_METERS = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 0;
  public static final double ZERO_POSITION_IN_METERS_FROM_GROUND = 0;

  public class tunning_values_intake {
      public static final double DUTY_CYCLE_INTAKE = 1;
      public static final double DUTY_CYCLE_EXPELL = -1;
      public static final double INTAKE_ROLLERS_IN_DEFAULT = 0;
    }

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 0;
    }


    public class setpoints {
      public static final double MAX_DISTANCE = 0;
      public static final double MIN_DISTANCE = 0;
      public static final double INTAKE_HOMED = 0;
      public static final double INTAKE_COLLECTING = 0;


    }
  }

