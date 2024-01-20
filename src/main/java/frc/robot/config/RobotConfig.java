package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotConfig {

  public DrivetrainConfig Drivetrain = new DrivetrainConfig(
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    new PIDConstants(0, 0, 0)
  );

  public PIDConstants kDrivePidConstants = new PIDConstants(0, 0, 0);

  public PIDConstants kSteeringPidConstants = new PIDConstants(0, 0, 0);

  public SwerveModuleConfig FrontLeftSwerveModuleConfig = new SwerveModuleConfig(
    "Front-Left",
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      -Drivetrain.TrackWidthMeters / 2,
      Drivetrain.WheelBaseMeters / 2
    )
  );

  public SwerveModuleConfig FrontRightSwerveModuleConfig = new SwerveModuleConfig(
    "Front-Right",
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      Drivetrain.TrackWidthMeters / 2,
      Drivetrain.WheelBaseMeters / 2
    )
  );

  public SwerveModuleConfig RearRightSwerveModuleConfig = new SwerveModuleConfig(
    "Rear-Right",
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      Drivetrain.TrackWidthMeters / 2,
      -Drivetrain.WheelBaseMeters / 2
    )
  );

  public SwerveModuleConfig RearLeftSwerveModuleConfig = new SwerveModuleConfig(
    "Rear-Left",
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      -Drivetrain.TrackWidthMeters / 2,
      -Drivetrain.WheelBaseMeters / 2
    )
  );
}
