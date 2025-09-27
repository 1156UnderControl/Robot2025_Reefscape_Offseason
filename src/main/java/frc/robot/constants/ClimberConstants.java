package frc.robot.constants;

public class ClimberConstants {
  public static final int ID_cageIntakeMotor = 18;
  public static final int ID_PivotMotor = 17;
  public static final int CAN_BUS_ID = 2;

  
    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE = 1.0;
      public static final double CLIMBED_ANGLE = 0.27;
      public static final double PREPARE_CLIMBED_ANGLE = -0.3;
      public static final double DEFAULT_ANGLE = 0;
    }
    public class PID {
      public static final double P = 500;
      public static final double I = 150;
      public static final double D = 0;
      public static final double F = 0;
      public static final double IZone = 0;
    }
}
  

