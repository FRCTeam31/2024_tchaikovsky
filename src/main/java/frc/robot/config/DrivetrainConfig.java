package frc.robot.config;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import prime.control.PrimePIDConstants;

public class DrivetrainConfig {

  // Physical properties
  public double TrackWidthMeters = 0;
  public double WheelBaseMeters = 0;
  public double WheelBaseCircumferenceMeters = 0;
  public double MaxSpeedMetersPerSecond = 0;
  public double MaxAccelerationMetersPerSecondSquared = 0;
  public double MaxAngularSpeedRadians = Math.PI;

  // CAN IDs
  public int PigeonId = 0;

  // Control properties
  public double LowGearScalar = 0.5;
  public boolean StartInHighGear = false;
  public double DriveDeadband = 0;
  public double DeadbandCurveWeight = 0;

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
    boolean startInHighGear,
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
    MaxAccelerationMetersPerSecondSquared =
      maxAccelerationMetersPerSecondSquared;
    MaxAngularSpeedRadians = maxAngularSpeedRadians;

    PigeonId = pigeonId;

    LowGearScalar = lowGearScalar;
    StartInHighGear = startInHighGear;
    DriveDeadband = driveDeadband;
    DeadbandCurveWeight = deadbandCurveWeight;

    DrivePID = drivePID;
    SteeringPID = steeringPID;
    SnapToPID = snapToPID;
    PathingTranslationPid = pathingTranslationPid;
    PathingRotationPid = pathingRotationPid;
  }

  public HolonomicPathFollowerConfig getHolonomicPathFollowerConfig() {
    return new HolonomicPathFollowerConfig(
      PathingTranslationPid.toPIDConstants(),
      PathingRotationPid.toPIDConstants(),
      MaxSpeedMetersPerSecond,
      MaxAccelerationMetersPerSecondSquared,
      new ReplanningConfig(false, false)
    );
  }
}
