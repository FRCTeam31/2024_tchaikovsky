package frc.robot.config;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import prime.control.PrimePIDConstants;

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
  public Pose3d LimelightPose;
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
    config.Drivetrain =
      new DrivetrainConfig(
        0.51181,
        0.67945,
        Math.PI * 0.7778174593052, // Wheelbase Circumference
        1,
        Units.feetToMeters(20), // Max Speed MPS
        Units.feetToMeters(15), // Max Acceleration MPS^2
        Math.PI * 3, // Max Angular Speed in Radians
        0.5,
        new PrimePIDConstants(0.019, 0, 0, 0, 0.091, 0, 0.05), // Drive PID
        new PrimePIDConstants(2, 0, 0), // Steering PID
        new PrimePIDConstants(6, 0, 0), // SnapTo PID
        new PrimePIDConstants(1.75, 0, 0), // Pathing Translation PID
        new PrimePIDConstants(0.5, 0, 0), // Pathing Rotation PID
        0.15,
        0.5
      );

    var wheelLocationAbsoluteX = config.Drivetrain.TrackWidthMeters / 2;
    var wheelLocationAbsoluteY = config.Drivetrain.WheelBaseMeters / 2;

    config.FrontLeftSwerveModule =
      new SwerveModuleConfig(
        "Front-Left",
        2,
        3,
        4,
        0.407,
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
        0.105,
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
        0.459,
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
        0.421,
        true,
        true,
        new Translation2d(-wheelLocationAbsoluteX, wheelLocationAbsoluteY),
        6.75,
        0.1016
      );

    config.Intake = new IntakeConfig(16, 15, 14, false, false, true, new PrimePIDConstants(0.05, 0, 0), 47, 4, 5);

    config.Shooter = new ShooterConfig(20, 19, false, false, 7, 6, 7);

    config.Climbers = new ClimbersConfig(18, 17, true, true, 0.5, -1, 2, 3, 8, 9, 10, 11);

    config.LimelightPose = new Pose3d(-0.2594356, 0, 0.5592318, new Rotation3d(0, 65, 180));

    config.LEDs = new LEDConfig(Port.kUSB, 1);

    config.PneumaticsModuleId = 30;

    return config;
  }

  @Override
  public String toString() {
    return Name;
  }
}
