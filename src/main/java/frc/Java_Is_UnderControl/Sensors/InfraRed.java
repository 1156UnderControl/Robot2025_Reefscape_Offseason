package frc.Java_Is_UnderControl.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class InfraRed implements SensorIO {
  private DigitalInput infraRed;
  private boolean inverted;

  public InfraRed(int port, boolean invert) {
    this.infraRed = new DigitalInput(port);
    this.inverted = invert;
  }

  @Override
  public boolean getBoolean() {
    boolean measurement = this.inverted ? !this.infraRed.get() : this.infraRed.get();
    return measurement;
  }

  @Override
  public int getBinary() {
    if (getBoolean()) {
      return 1;
    } else {
      return 0;
    }
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = this.infraRed.getChannel() >= 0;
    inputs.value = getBoolean();
    inputs.inverted = this.inverted;
    inputs.timestampSec = System.currentTimeMillis() / 1000.0; // Update timestamp in seconds
  }
}
