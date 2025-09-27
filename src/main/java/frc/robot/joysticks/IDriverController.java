package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriverController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  Trigger turboActivate();

  Trigger slowActivate();

  boolean notUsingJoystick();

  Trigger y();

  Trigger b();

  Trigger a();

  Trigger x();

  Trigger leftBumper();

  Trigger rightBumper();

  Trigger start();

  Trigger miniLeft();

  Trigger isForcingDriverControl();

  Trigger leftArrow();

  Trigger rightArrow();

  Trigger upArrow();

  Trigger downArrow();

  Trigger setHasAlgae();

  Trigger setHasCoral();

  Trigger setNoObject();
}
