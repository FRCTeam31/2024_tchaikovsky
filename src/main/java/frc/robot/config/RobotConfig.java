package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotConfig {

  public String Name;
  public DrivetrainConfig Drivetrain;
  public SwerveModuleConfig FrontLeftSwerveModule;
  public SwerveModuleConfig FrontRightSwerveModule;
  public SwerveModuleConfig RearRightSwerveModule;
  public SwerveModuleConfig RearLeftSwerveModule;
  public IntakeConfig Intake;
  public ShooterConfig Shooter;
  public ClimbersConfig Climbers;
  public LEDConfig LEDs;
  public int PneumaticsModuleId;

  public RobotConfig() {
    Name = "[none]";
  }

  public RobotConfig(String name) {
    Name = name;
  }

  /**
   * Gets a default instance of a RobotConfig with all properties set to 0
   * @return A default instance of a RobotConfig
   */
  public static RobotConfig getDefault() {
    var config = new RobotConfig("Default Config");
    config.Drivetrain = new DrivetrainConfig();
    config.Intake = new IntakeConfig();
    config.Shooter = new ShooterConfig();
    config.Climbers = new ClimbersConfig();
    config.LEDs = new LEDConfig();
    config.PneumaticsModuleId = 30;

    var wheelLocationAbsoluteX = config.Drivetrain.TrackWidthMeters / 2;
    var wheelLocationAbsoluteY = config.Drivetrain.WheelBaseMeters / 2;

    config.FrontLeftSwerveModule =
      new SwerveModuleConfig(
        "Front-Left",
        2,
        3,
        4,
        0.407 + 0.25,
        true,
        true,
        new Translation2d(wheelLocationAbsoluteX, wheelLocationAbsoluteY),
        6.75,
        0.1016
      );

    config.FrontRightSwerveModule =
      new SwerveModuleConfig(
        "Front-Right",
        5,
        6,
        7,
        0.105 + 0.25,
        true,
        true,
        new Translation2d(wheelLocationAbsoluteX, -wheelLocationAbsoluteY),
        6.75,
        0.1016
      );

    config.RearRightSwerveModule =
      new SwerveModuleConfig(
        "Rear-Right",
        8,
        9,
        10,
        0.459 + 0.25,
        true,
        true,
        new Translation2d(-(wheelLocationAbsoluteX), -(wheelLocationAbsoluteY)),
        6.75,
        0.1016
      );

    config.RearLeftSwerveModule =
      new SwerveModuleConfig(
        "Rear-Left",
        11,
        12,
        13,
        0.421 + 0.25,
        true,
        true,
        new Translation2d(-wheelLocationAbsoluteX, wheelLocationAbsoluteY),
        6.75,
        0.1016
      );

    return config;
  }

  @Override
  public String toString() {
    return Name;
  }
}
