package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final int ID_intakeWheelsMotor = 0;
  public static final int ID_intakePivotMotor = 0;
  public static final int ID_indexerMotor = 0;
  public static final int intakeBusID = 0;
  public static final String intakeWheelsMotorName = "Intake Wheels";
  public static final String intakePivotMotorName = "Intake Pivot";
  public static final String indexerMotorName = "Indexer";
  public static final int port_IR = 0;
  public static final Pose2d BREAK_POINT_POSE_PIVOT_CLOSED = new Pose2d(0.517806, 0.330912, new Rotation2d());

  public class tunning_values_intake {
      public static final double INTAKE_SPEED = 1;
      public static final double EXPELL_SPEED = -1;
      public static final double STOP_SPEED = 0;
      public static final double INDEXER_SPEED = 0.4;
      
    }

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 0;
    }


    public class setpoints {
      public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_ANGLE = 0;
      public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_METERS_PER_SECOND = 0;
      public static final double ZERO_POSITION_IN_METERS_FROM_GROUND = 0;
      public static final double MAX_DISTANCE = 0;
      public static final double MIN_DISTANCE = 0;
      public static final double INTAKE_ANGLE_HOMED = 0;
      public static final double INTAKE_ANGLE_COLLECTING = 0;
    }
  }

