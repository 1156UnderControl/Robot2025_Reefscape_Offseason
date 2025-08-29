package frc.Java_Is_UnderControl.Sensors;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {

    @AutoLog
    public static class SensorIOInputs {
        public boolean connected = true;
        public boolean value = false;
        public boolean inverted = false;
        public double timestampSec = 0.0;
    }

    void updateInputs(SensorIOInputs inputs);
    boolean getBoolean();
    int getBinary();
}
