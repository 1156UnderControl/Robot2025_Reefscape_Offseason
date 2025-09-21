package frc.robot.constants;

public class EndEffectorConstants {
  public static final int endEffectorBusID = 4;
  public static final int ID_endEffectorMotor = 16;
  public static final String endEffectorMotorName = "End Effector Motor";
  public static final int Port_coralInfraRed = 1;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM = 1;

  public class tunning_values_endeffector {
    public static final double APPLIED_OUTPUT_ERROR_ALLOWED = 0.1;
    public static final double VELOCITY_TO_DETECT_RPM_FALL = 6000;
    public static final double MINIMUM_VELOCITY_FOR_DETECTION = 6000;

    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE_CORAL = 1;
      public static final double DUTY_CYCLE_INTAKE_ALGAE = 0;
      public static final double DUTY_CYCLE_EXPELL_CORAL = -0.3;
      public static final double DUTY_CYCLE_EXPELL_ALGAE = 0;
      public static final double DUTY_CYCLE_EXPELL_CORAL_L1 = 0;
      public static final double DUTY_CYCLE_HOLDING_ALGAE = 0;
      public static final double DUTY_CYCLE_HOLDING_ALGAE_DURING_MOVEMENT = 0;
    }
  }
}
