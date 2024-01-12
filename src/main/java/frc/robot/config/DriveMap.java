package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import prime.utilities.CTREConverter;

public class DriveMap {

  // Physical measurements
  public static final double kRobotTrackWidthMeters = 0;
  public static final double kRobotWheelBaseMeters = 0;
  public static final double kRobotWheelBaseCircumferenceMeters = 0;
  public static final double kDriveWheelDiameterMeters = 0;
  public static final byte kDriveMotorOutputTeeth = 0;
  public static final byte kDriveMotorDrivenGearTeeth = 0;

  // Calculated values
  public static final double kDriveGearRatio =
    kDriveMotorDrivenGearTeeth / kDriveMotorOutputTeeth;
  public static final double kDriveWheelCircumference =
    Math.PI * kDriveWheelDiameterMeters;

  // Locations of each wheel in terms of x, y translation in meters from the
  // middle of the robot
  static final double halfWheelBase = DriveMap.kRobotWheelBaseMeters / 2;
  static final double halfTrackWidth = DriveMap.kRobotTrackWidthMeters / 2;

  // Measured SysId values
  public static final double driveKs = 0;
  public static final double driveKv = 0;
  public static final double driveKa = 0;
  public static final String kDrivePidConstantsName =
    "SwerveModule drive PID Constants";
  public static PIDConstants kDrivePidConstants = new PIDConstants(0, 0, 0);

  public static final String kSteeringPidConstantsName =
    "SwerveModule steering PID Constants";

  public static PIDConstants kSteeringPidConstants = new PIDConstants(0, 0, 0);
  public static final double kD_min = 0;
  // public Static;
  public static double kSteeringGearRatio = 0;

  // Pigeon
  public static final int kPigeonId = 0;
  public static final String kCANivoreBusName = "Team31CANivore";

  // Gear ratios
  public static byte driveMotorOutputTeeth = 0;
  public static byte driveMotorDriveGearTeeth = 0;
  public static int falconTotalSensorUnits = 0;
  public static final double kDriveMaxSpeedMetersPerSecond = CTREConverter.falconToMPS(
    22000,
    kDriveWheelCircumference,
    kDriveGearRatio
  ); // 16.2ft
  public static final double kDriveMaxAccelerationMetersPerSecondSquared =
    kDriveMaxSpeedMetersPerSecond * 0;

  // per second in meters per second
  public static final double kDriveMaxAngularSpeed = Math.PI;
  public static final double kLowGearCoefficient = 0;

  // Snap To Gyro Angle PID Constants

  // public static final double kSnapToGyroAngle_kP = kDriveMaxAngularSpeed / 4;
  public static final double kSnapToGyroAngle_kP = 0;
  public static final double kSnapToGyroAngle_kI = 0;
  public static final double kSnapToGyroAngle_kD = 0;

  public static final SwerveModuleConfig FrontLeftSwerveModuleConfig = new SwerveModuleConfig(
    "Front-Left",
    0,
    0,
    0,
    0,
    false,
    kDrivePidConstants,
    kSteeringPidConstants,
    new Translation2d(-halfTrackWidth, halfWheelBase)
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
    new Translation2d(halfTrackWidth, halfWheelBase)
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
    new Translation2d(halfTrackWidth, -halfWheelBase)
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
    new Translation2d(-halfTrackWidth, -halfWheelBase)
  );

  public static final int encoderId = 0;
}
