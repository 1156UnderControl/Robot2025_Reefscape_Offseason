package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements IDriverController, IOperatorController {
  private static ControlBoard mInstance = null;

  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }

    return mInstance;
  }

  IDriverController mDriverController;
  IOperatorController mOperatorController;

  private ControlBoard() {
    mDriverController = DriverController.getInstance();
    mOperatorController = OperatorController.getInstance();
  }

  @Override
  public double getXtranslation() {
    return mDriverController.getXtranslation();
  }

  @Override
  public double getYtranslation() {
    return mDriverController.getYtranslation();
  }

  @Override
  public double getCOS_Joystick() {
    return mDriverController.getCOS_Joystick();
  }

  @Override
  public double getSIN_Joystick() {
    return mDriverController.getSIN_Joystick();
  }

  @Override
  public Trigger turboActivate() {
    return mDriverController.turboActivate();
  }

  @Override
  public boolean notUsingJoystick() {
    return mDriverController.notUsingJoystick();
  }

  @Override
  public Trigger a() {
    return mDriverController.a();
  }

  @Override
  public Trigger y() {
    return mDriverController.y();
  }

  @Override
  public Trigger x() {
    return mDriverController.x();
  }

  @Override
  public Trigger b() {
    return mDriverController.b();
  }

  @Override
  public Trigger leftBumper() {
    return mDriverController.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return mDriverController.rightBumper();
  }

  @Override
  public Trigger miniLeft() {
    return mDriverController.miniLeft();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return mDriverController.isForcingDriverControl();
  }

  @Override
  public Trigger setHasAlgae() {
    return mDriverController.setHasAlgae();
  }

  @Override
  public Trigger setHasCoral() {
    return mDriverController.setHasCoral();
  }

  @Override
  public Trigger setNoObject() {
    return mDriverController.setNoObject();
  }

  @Override
  public Trigger start() {
    return mDriverController.start();
  }

  @Override
  public Trigger leftArrow() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'leftArrow'");
  }

  @Override
  public Trigger rightArrow() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rightArrow'");
  }

  @Override
  public Trigger upArrow() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'upArrow'");
  }

  @Override
  public Trigger downArrow() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'downArrow'");
  }
}
