package frc.Java_Is_UnderControl.Logging.enhanced_loggers;

import edu.wpi.first.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomDoubleLogger extends DoubleLogEntry {

  private static boolean isFmsMatch;

  private String name;

  private double loggedValue;

  public CustomDoubleLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomDoubleLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = 1; // Set to something different than default for initial logging
    this.append(0);
  }

  @Override
  public void append(double value) {
    if (DriverStation.isEnabled() && value != this.loggedValue) {
      this.loggedValue = value;
      super.append(value);
      if (!CustomDoubleLogger.isFmsMatch) {
        SmartDashboard.putNumber(this.name, value);
      }
    }
  }
}
