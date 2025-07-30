package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Control.PIDConfig;
import frc.Java_Is_UnderControl.Swerve.BaseSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.Configs.BaseSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.Configs.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Swerve.Constants.SwerveConstants;
import frc.robot.Joysticks.DriverController;
import frc.robot.generated.TunerConstants;
import java.util.HashMap;

public class SwerveSubsystem extends SubsystemBase {
  private final BaseSwerveSubsystem swerve;
  private final HashMap<
          String,
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      moduleConstants;
  private final DriverController driverController;

  public SwerveSubsystem() {
    this.moduleConstants = new HashMap<>();

    this.moduleConstants.put(SwerveConstants.FRONT_LEFT_MODULE_NAME, TunerConstants.FrontLeft);

    this.moduleConstants.put(SwerveConstants.FRONT_RIGHT_MODULE_NAME, TunerConstants.FrontRight);

    this.moduleConstants.put(SwerveConstants.BACK_LEFT_MODULE_NAME, TunerConstants.BackLeft);

    this.moduleConstants.put(SwerveConstants.BACK_RIGHT_MODULE_NAME, TunerConstants.BackRight);

    this.driverController = DriverController.getInstance();

    this.swerve =
        new BaseSwerveSubsystem(
            new BaseSwerveConfig(
                0.9,
                new SwervePathPlannerConfig(
                    new PIDConstants(0.1, 0.1, 0.1),
                    new PIDConstants(0.1, 0.1, 0.1),
                    new PathConstraints(5, 5, 2, 1)),
                new PIDConfig(0.1, 0.1, 0.1, 0.1)),
            this.moduleConstants);
  }

  public void driveFieldOrientedLockedJoystickAngle() {
    System.out.println("Comecei a rodar o comando");
    ChassisSpeeds joystickSpeeds =
        swerve.inputsToChassisSpeeds(
            driverController.getXtranslation(), driverController.getYtranslation());
    this.swerve.driveFieldOrientedLockedJoystickAngle(
        joystickSpeeds, driverController.getCOS_Joystick(), driverController.getSIN_Joystick());
  }
}
