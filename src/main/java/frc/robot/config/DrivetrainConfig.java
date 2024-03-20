package frc.robot.config;

import prime.control.PrimePIDConstants;

public class DrivetrainConfig {

  // Physical properties
  public double TrackWidthMeters;
  public double WheelBaseMeters;
  public double WheelBaseCircumferenceMeters;
  public double MaxSpeedMetersPerSecond;
  public double MaxAccelerationMetersPerSecondSquared;
  public double MaxAngularSpeedRadians;

  // CAN IDs
  public int PigeonId;

  // Control properties
  public double LowGearScalar;
  public double DriveDeadband;
  public double DeadbandCurveWeight;

  // PID configs
  public PrimePIDConstants DrivePID;
  public PrimePIDConstants SteeringPID;
  public PrimePIDConstants SnapToPID;
  public PrimePIDConstants PathingTranslationPid;
  public PrimePIDConstants PathingRotationPid;

  public DrivetrainConfig(
    double trackWidthMeters,
    double wheelBaseMeters,
    double wheelBaseCircumferenceMeters,
    int pigeonId,
    double maxSpeedMetersPerSecond,
    double maxAccelerationMetersPerSecondSquared,
    double maxAngularSpeedRadians,
    double lowGearScalar,
    PrimePIDConstants drivePID,
    PrimePIDConstants steeringPID,
    PrimePIDConstants snapToPID,
    PrimePIDConstants pathingTranslationPid,
    PrimePIDConstants pathingRotationPid,
    double driveDeadband,
    double deadbandCurveWeight
  ) {
    TrackWidthMeters = trackWidthMeters;
    WheelBaseMeters = wheelBaseMeters;
    WheelBaseCircumferenceMeters = wheelBaseCircumferenceMeters;
    MaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    MaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
    MaxAngularSpeedRadians = maxAngularSpeedRadians;

    PigeonId = pigeonId;

    LowGearScalar = lowGearScalar;
    DriveDeadband = driveDeadband;
    DeadbandCurveWeight = deadbandCurveWeight;

    DrivePID = drivePID;
    SteeringPID = steeringPID;
    SnapToPID = snapToPID;
    PathingTranslationPid = pathingTranslationPid;
    PathingRotationPid = pathingRotationPid;
  }
}
