package frc.robot.constants;

public class EndEffectorConstants {
  public static final int endEffectorBusID = 4;
  public static final int ID_endEffectorMotor = 16;
  public static final String endEffectorMotorName = "End Effector Motor";
  public static final int Port_coralInfraRed = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM = 1;

  public class tunning_values_endeffector {
    public static final double VELOCITY_FALL_FOR_INTAKE_DETECTION = 0;
    public static final double MIN_VELOCITY_FOR_INTAKE_DETECTION_INITIALIZE = 0;
    public static final double SLOW_VELOCITY_FOR_INTAKE_ALGAE_DETECTION = 0;
    public static final double SLOW_VELOCITY_FOR_INTAKE_ALGAE_DETECTION_IN_AUTO = 0;

    public class setpoints {
      public static final double DUTY_CYCLE_INTAKE_CORAL = 0;
      public static final double DUTY_CYCLE_INTAKE_ALGAE = 0;
      public static final double DUTY_CYCLE_EXPELL_CORAL = 0;
      public static final double DUTY_CYCLE_EXPELL_ALGAE = 0;
      public static final double DUTY_CYCLE_EXPELL_CORAL_L1 = 0;
      public static final double DUTY_CYCLE_HOLDING_ALGAE = 0;
      public static final double DUTY_CYCLE_HOLDING_ALGAE_DURING_MOVEMENT = 0;
    }
  }
}
