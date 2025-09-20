package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomBooleanLogger extends BooleanLogEntry {

  private static boolean isFmsMatch;

  private String name;

  private boolean loggedValue;

  public CustomBooleanLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomBooleanLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = true; // Set to something different than default for initial logging
    this.append(false);
  }

  @Override
  public void append(boolean value) {
    if (DriverStation.isEnabled() && value != this.loggedValue) {
      this.loggedValue = value;
      super.append(value);
      if (!CustomBooleanLogger.isFmsMatch) {
        SmartDashboard.putBoolean(this.name, value);
      }
    }
  }

}
