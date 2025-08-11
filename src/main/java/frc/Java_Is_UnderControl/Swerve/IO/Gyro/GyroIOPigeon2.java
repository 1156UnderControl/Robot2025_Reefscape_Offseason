package frc.Java_Is_UnderControl.Swerve.IO.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.Java_Is_UnderControl.Swerve.IO.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.generated.TunerConstants;

import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final Pigeon2Configuration configuration;
  private GyroIOInputs inputs;

  public GyroIOPigeon2() {
    this.pigeon =
        new Pigeon2(
            TunerConstants.DrivetrainConstants.Pigeon2Id,
            TunerConstants.DrivetrainConstants.CANBusName);
    this.configuration = new Pigeon2Configuration();
    this.pigeon.optimizeBusUtilization();
    // this will need to be fixed
    this.setYaw(90);
    this.yaw = pigeon.getYaw();
    this.yaw.setUpdateFrequency(SwerveConstants.ODOMETRY_FREQUENCY);

    this.yawVelocity = pigeon.getAngularVelocityZWorld();
    this.yawVelocity.setUpdateFrequency(SwerveConstants.LOW_FREQUENCY);

    this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    this.inputs = inputs;
    this.inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    this.inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    this.inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    this.inputs.odometryYawTimestamps =
        this.yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    this.inputs.odometryYawPositions =
        this.yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    this.yawTimestampQueue.clear();
    this.yawPositionQueue.clear();
  }

  @Override
  public GyroIOInputs getInputs() {
    return this.inputs;
  }

  @Override
  public void setYaw(double yaw) {
    this.pigeon.getConfigurator().setYaw(yaw);
    this.pigeon.getConfigurator().apply(configuration);
  }
}
