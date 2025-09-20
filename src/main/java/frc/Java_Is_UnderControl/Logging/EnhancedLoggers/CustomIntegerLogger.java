package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomIntegerLogger extends  IntegerLogEntry{

  private static boolean isFmsMatch;

  private String name;

  private long loggedValue;

  public CustomIntegerLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomIntegerLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = 1; // Set to something different than default for initial logging
    this.append(0);
  }

  @Override
  public void append(long value) {
    if (DriverStation.isEnabled() && value != this.loggedValue) {
      this.loggedValue = value;
      super.append(value);
      if (!CustomIntegerLogger.isFmsMatch) {
        SmartDashboard.putNumber(this.name, value);
      }
    }
  }
}
