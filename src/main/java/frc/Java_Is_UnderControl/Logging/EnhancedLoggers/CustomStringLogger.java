package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomStringLogger extends  StringLogEntry{

  private static boolean isFmsMatch;

  private String name;

  private String loggedValue;

  public CustomStringLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomStringLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = "a"; // Set to something different than default for initial logging
    this.append("");
  }

  @Override
  public void append(String value) {
    if (DriverStation.isEnabled() && !value.equals(this.loggedValue)) {
      this.loggedValue = value;
      super.append(value);
      if (!CustomStringLogger.isFmsMatch) {
        SmartDashboard.putString(this.name, value);
      }
    }
  }

}
