package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Java_Is_UnderControl.Util.JoystickUtil;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;
  private final double deadBand;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private DriverController() {
    this.deadBand = JoystickConstants.JOYSTICK_DEAD_BAND;
    driverController = new CommandXboxController(0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(
              performAllianceInputDirectionCorrection(driverController.getLeftX()), this.deadBand)
          * 1;
    }
    if(this.slowActivate().getAsBoolean()){
      return -MathUtil.applyDeadband(
              performAllianceInputDirectionCorrection(driverController.getLeftX()), this.deadBand)
          * 0.12;
    }
    return -MathUtil.applyDeadband(
            performAllianceInputDirectionCorrection(driverController.getLeftX()), this.deadBand)
        * 0.7;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(
              performAllianceInputDirectionCorrection(driverController.getLeftY()), this.deadBand)
          * 1.0;
    }
    if(this.slowActivate().getAsBoolean()){
      return -MathUtil.applyDeadband(
              performAllianceInputDirectionCorrection(driverController.getLeftY()), this.deadBand)
          * 0.12;
    }
    return -MathUtil.applyDeadband(
            performAllianceInputDirectionCorrection(driverController.getLeftY()), this.deadBand)
        * 0.7;
  }

  @Override
  public double getCOS_Joystick() {
    return -performAllianceInputDirectionCorrection(driverController.getRightX());
  }

  @Override
  public double getSIN_Joystick() {
    return -performAllianceInputDirectionCorrection(driverController.getRightY());
  }

  @Override
  public Trigger turboActivate() {
    return driverController.rightTrigger(0.2);
  }

  @Override
  public Trigger slowActivate() {
    return driverController.leftTrigger(0.2);
  }

  @Override
  public boolean notUsingJoystick() {
    return JoystickUtil.inRange(getCOS_Joystick(), -this.deadBand, this.deadBand)
        && JoystickUtil.inRange(getSIN_Joystick(), -this.deadBand, this.deadBand);
  }

  @Override
  public Trigger a() {
    return driverController.a();
  }

  @Override
  public Trigger y() {
    return driverController.y();
  }

  @Override
  public Trigger x() {
    return driverController.x();
  }

  @Override
  public Trigger b() {
    return driverController.b();
  }

  @Override
  public Trigger leftBumper() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return driverController.rightBumper();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return driverController.leftTrigger(0.4);
  }

  @Override
  public Trigger start() {
    return driverController.start();
  }

  @Override
  public Trigger miniLeft() {
    return driverController.button(6);
  }

  @Override
  public Trigger leftArrow() {
    return driverController.povLeft();
  }

  @Override
  public Trigger rightArrow() {
      return driverController.povRight();
  }

  @Override
  public Trigger upArrow() {
      return driverController.povUp();
  }

  @Override
  public Trigger downArrow() {
      return driverController.povDown();
  }

  @Override
  public Trigger setHasAlgae() {
    return driverController.leftStick().and(driverController.rightStick().negate());
  }

  @Override
  public Trigger setHasCoral() {
    return driverController.rightStick().and(driverController.leftStick().negate());
  }

  @Override
  public Trigger setNoObject() {
    return driverController.leftStick().and(driverController.rightStick());
  }

  private double performAllianceInputDirectionCorrection(Double value) {
    Alliance alliance =
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Red;
    if (alliance == Alliance.Red) {
      return -value;
    }
    return value;
  }
}
