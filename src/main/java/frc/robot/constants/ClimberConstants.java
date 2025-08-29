package frc.robot.constants;

public class ClimberConstants {
  public static final int ID_cageIntakeMotor = 2;
  public static final int ID_climberArmMotor = 10;

  public class tunning_values_intake {
    public static final double VELOCITY_FALL_FOR_CAGE_INTAKE_DETECTION = 100;

    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE = 1.0;
      public static final double DUTY_CYCLE_EXPELL = -1.0;
    }

  }

  public class tunning_values_arm {
    public class setpoints {
      public static final double MIN_ANGLE = -0.2;
      public static final double MAX_ANGLE = 0.54;
      public static final double STOW_ANGLE = 0.05;
      public static final double INTAKE_CAGE_ANGLE = 0.49;
    }

    public class PID {
      public static final double P = 200;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0;
      public static final double IZone = 0;
    }
  }
}
