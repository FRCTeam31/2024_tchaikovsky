package frc.robot.subsystems.drive;

import frc.robot.config.RobotConfig;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.drive.gyroIO.*;
import frc.robot.subsystems.drive.limelightIO.*;
import frc.robot.subsystems.drive.swerveIO.*;

public class DrivetrainFactory {

  public static Drivetrain build(RobotConfig config, PwmLEDs leds, boolean isReal) {
    return new Drivetrain(
      config,
      leds,
      GyroIOFactory.build(config, isReal),
      LimelightIOFactory.build(),
      SwerveIOFactory.build(config, isReal)
    );
  }

  public class GyroIOFactory {

    public static IGyro build(RobotConfig config, boolean isReal) {
      if (isReal) {
        return new GyroPigeon2(config.Drivetrain.PigeonId);
      } else {
        return new GyroSim();
      }
    }
  }

  public class LimelightIOFactory {

    public static ILimelight build() {
      return new LimelightReal();
    }
  }

  public class SwerveIOFactory {

    public static ISwerveController build(RobotConfig config, boolean isReal) {
      if (isReal) {
        return new SwerveControllerReal(config);
      } else {
        return new SwerveControllerSim(config);
      }
    }
  }
}
