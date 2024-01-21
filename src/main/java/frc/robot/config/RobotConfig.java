package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotConfig {

  public DrivetrainConfig Drivetrain;

  public SwerveModuleConfig FrontLeftSwerveModule;
  public SwerveModuleConfig FrontRightSwerveModule;
  public SwerveModuleConfig RearRightSwerveModule;
  public SwerveModuleConfig RearLeftSwerveModule;

  /**
   * Gets a default instance of a RobotConfig with all properties set to 0
   * @return A default instance of a RobotConfig
   */
  public static RobotConfig getDefault() {
    var config = new RobotConfig();
    config.Drivetrain =
      new DrivetrainConfig(
        0.55,
        0.55,
        Math.PI * 0.7778174593052,
        1,
        Units.feetToMeters(10),
        Units.feetToMeters(5),
        Math.PI,
        0.3,
        false,
        new double[] { 0.1, 0, 0 },
        new double[] { 2, 0, 0 },
        new double[] { 0, 0, 0 }
      );

    config.FrontLeftSwerveModule =
      new SwerveModuleConfig(
        "Front-Left",
        2,
        3,
        4,
        0.164551,
        true,
        true,
        new Translation2d(
          -config.Drivetrain.TrackWidthMeters / 2,
          config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.FrontRightSwerveModule =
      new SwerveModuleConfig(
        "Front-Right",
        5,
        6,
        7,
        0.350098,
        false,
        true,
        new Translation2d(
          config.Drivetrain.TrackWidthMeters / 2,
          config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.RearRightSwerveModule =
      new SwerveModuleConfig(
        "Rear-Right",
        8,
        9,
        10,
        0.717773,
        false,
        true,
        new Translation2d(
          config.Drivetrain.TrackWidthMeters / 2,
          -config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.RearLeftSwerveModule =
      new SwerveModuleConfig(
        "Rear-Left",
        11,
        12,
        13,
        0.181152,
        true,
        true,
        new Translation2d(
          -config.Drivetrain.TrackWidthMeters / 2,
          -config.Drivetrain.WheelBaseMeters / 2
        )
      );

    return config;
  }
}
