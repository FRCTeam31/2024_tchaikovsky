package frc.robot.config;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
        0.55,
        0.55,
        Math.PI * 0.7778174593052, // Wheelbase Circumference
        1,
        Units.feetToMeters(15.7), // Max Speed MPS
        Units.feetToMeters(10), // Max Acceleration MPS^2
        Math.PI * 2, // Max Angular Speed in Radians
        0.5,
        false,
        new PrimePIDConstants(0.019, 0, 0, 0, 0.122), // Drive PID
        new PrimePIDConstants(2, 0, 0), // Steering PID
        new PrimePIDConstants(0, 0, 0), // SnapTo PID,
        new PrimePIDConstants(0.001, 0, 0), // Pathing Translation PID
        new PrimePIDConstants(1, 0, 0), // Pathing Rotation PID
        0.15,
        0.5
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
        ),
        6.75,
        0.102
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
        ),
        6.75,
        0.102
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
        ),
        6.75,
        0.102
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
        ),
        6.75,
        0.102
      );

    config.Intake =
      new IntakeConfig(
        16,
        15,
        14,
        false,
        false,
        true,
        new PrimePIDConstants(0.032, 0, 0),
        4,
        48
      );

    config.Shooter = new ShooterConfig(20, 19, false, false, 21, 0, 22, 1, 4);

    config.Climbers =
      new ClimbersConfig(18, 17, true, true, 0.2, -1, 2, 3, 0, 1, 180, 150);

    config.LimelightPose = new Pose3d(); // TODO: Find out what the pose (translation & rotation) of the camera lens is from the robot center -- ask Arseni to get it from the CAD model

    return config;
  }

  @Override
  public String toString() {
    return Name;
  }
}
