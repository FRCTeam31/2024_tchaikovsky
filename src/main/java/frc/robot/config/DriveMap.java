package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveMap {

  public static final DrivetrainConfig DrivetrainConfig = new DrivetrainConfig(
    0.55,
    0.55,
    0.284,
    1,
    5,
    2,
    Math.PI,
    0.3,
    new PIDConstants(0, 0, 0)
  );

  public static final PIDConstants kDrivePidConstants = new PIDConstants(
    0,
    0,
    0
  );

  public static final PIDConstants kSteeringPidConstants = new PIDConstants(
    0,
    0,
    0
  );

  public static final SwerveModuleConfig FrontLeftSwerveModuleConfig = new SwerveModuleConfig(
    "Front-Left",
    2,
    3,
    4,
    0.16,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      -DrivetrainConfig.TrackWidthMeters / 2,
      DrivetrainConfig.WheelBaseMeters / 2
    )
  );

  public static final SwerveModuleConfig FrontRightSwerveModuleConfig = new SwerveModuleConfig(
    "Front-Right",
    5,
    6,
    7,
    0.356,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      DrivetrainConfig.TrackWidthMeters / 2,
      DrivetrainConfig.WheelBaseMeters / 2
    )
  );

  public static final SwerveModuleConfig RearRightSwerveModuleConfig = new SwerveModuleConfig(
    "Rear-Right",
    8,
    9,
    10,
    0.356,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      DrivetrainConfig.TrackWidthMeters / 2,
      -DrivetrainConfig.WheelBaseMeters / 2
    )
  );

  public static final SwerveModuleConfig RearLeftSwerveModuleConfig = new SwerveModuleConfig(
    "Rear-Left",
    11,
    12,
    13,
    0.16,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      -DrivetrainConfig.TrackWidthMeters / 2,
      -DrivetrainConfig.WheelBaseMeters / 2
    )
  );
}
