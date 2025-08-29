package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final int ID_intakeMotor = 5;
  public static final int port_IR = 0;
  public static final Pose2d BREAK_POINT_POSE_PIVOT_CLOSED = new Pose2d(0.517806, 0.330912, new Rotation2d());

  public class tunning_values_intake {
    public class setpoints {
      public static final double SPEED_INTAKE = 1;
      public static final double SPEED_EXPELL = -1;
    }
  }
}
