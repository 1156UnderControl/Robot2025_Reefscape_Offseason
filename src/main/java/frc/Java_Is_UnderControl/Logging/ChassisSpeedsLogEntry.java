package frc.Java_Is_UnderControl.Logging;

import edu.wpi.first.datalog.DataLog;
import edu.wpi.first.datalog.DoubleArrayLogEntry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsLogEntry {
    private  DoubleArrayLogEntry baseLogger;

    public ChassisSpeedsLogEntry(DataLog log, String name) {
        this.baseLogger = new DoubleArrayLogEntry(log, name);
    }

    public void append(ChassisSpeeds chassisSpeeds){
        double[] data = new double[3];
        data[0] = chassisSpeeds.vx;
        data[1] = chassisSpeeds.vy;
        data[2] = chassisSpeeds.omega;
        this.baseLogger.append(data);
    }
}
