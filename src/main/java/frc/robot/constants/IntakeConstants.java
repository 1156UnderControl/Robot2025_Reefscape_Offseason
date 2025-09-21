package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final int ID_intakeWheelsMotor = 20;
  public static final int ID_intakePivotMotor = 19;
  public static final int ID_indexerMotor = 21;
  public static final int intakeBusID = 0;
  public static final String intakeWheelsMotorName = "Intake Wheels";
  public static final String intakePivotMotorName = "Intake Pivot";
  public static final String indexerMotorName = "Indexer";
  public static final int port_IR = 0;
  public static final double INTAKE_HEIGHT_FROM_GROUND_INTAKING = 0.330912;
  public static final double INTAKE_HEIGHT_FROM_GROUND_HOMED = 0.5842;

  public class tunning_values_intake {
      public static final double INTAKE_SPEED = 1;
      public static final double EXPELL_SPEED = -1;
      public static final double STOP_SPEED = 0;
      public static final double INDEXER_SPEED = 0.7;
      public static final double ANGLE_ERROR_ALLOWED = 1;
      
      public class PID {
        public static final double P = 0.045;
        public static final double I = 0.0002;
        public static final double D = 0;
        public static final double arbFF = 0;
        public static final double IZone = 1;
        public static final double MAX_ACCELERATION = 2500;
        public static final double MAX_VELOCITY = 400;
      }
  
      public class setpoints {
        public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_ANGLE = 0.09;
        public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_ANGLE_PER_SECOND = 1;
        public static final double ZERO_POSITION_IN_ANGLE = 0;
        public static final double MAX_ANGLE = 0;
        public static final double MIN_ANGLE = 0;
        public static final double INTAKE_ANGLE_HOMED = 45;
        public static final double INTAKE_ANGLE_FOR_NOT_TOUCHING_PIVOT = 30;
        public static final double INTAKE_ANGLE_COLLECTING = 0;
  
      }
    }
  }

