package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveMap {

  public static final DrivetrainConfig DrivetrainConfig = new DrivetrainConfig(
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
    0,
    0,
    0,
    0,
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
    0,
    0,
    0,
    0,
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
    0,
    0,
    0,
    0,
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
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(
      -DrivetrainConfig.TrackWidthMeters / 2,
      -DrivetrainConfig.WheelBaseMeters / 2
    )
  );

  public static final int encoderId = 0;
  public static final double kLowGearCoefficient = 0.3;
}
