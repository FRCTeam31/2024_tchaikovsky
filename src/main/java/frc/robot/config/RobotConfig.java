package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;

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
        0,
        0,
        0,
        0,
        0,
        0,
        Math.PI,
        0.5,
        new double[] { 0, 0, 0 },
        new double[] { 0, 0, 0 },
        new double[] { 0, 0, 0 }
      );

    config.FrontLeftSwerveModule =
      new SwerveModuleConfig(
        "Front-Left",
        0,
        0,
        0,
        0,
        false,
        new Translation2d(
          -config.Drivetrain.TrackWidthMeters / 2,
          config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.FrontRightSwerveModule =
      new SwerveModuleConfig(
        "Front-Right",
        0,
        0,
        0,
        0,
        false,
        new Translation2d(
          config.Drivetrain.TrackWidthMeters / 2,
          config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.RearRightSwerveModule =
      new SwerveModuleConfig(
        "Rear-Right",
        0,
        0,
        0,
        0,
        false,
        new Translation2d(
          config.Drivetrain.TrackWidthMeters / 2,
          -config.Drivetrain.WheelBaseMeters / 2
        )
      );

    config.RearLeftSwerveModule =
      new SwerveModuleConfig(
        "Rear-Left",
        0,
        0,
        0,
        0,
        false,
        new Translation2d(
          -config.Drivetrain.TrackWidthMeters / 2,
          -config.Drivetrain.WheelBaseMeters / 2
        )
      );

    return config;
  }
}
