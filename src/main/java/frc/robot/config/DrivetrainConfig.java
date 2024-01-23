package frc.robot.config;

public class DrivetrainConfig {

  public double TrackWidthMeters = 0;
  public double WheelBaseMeters = 0;
  public double WheelBaseCircumferenceMeters = 0;

  public int PigeonId = 0;

  public double MaxSpeedMetersPerSecond = 0;
  public double MaxAccelerationMetersPerSecondSquared = 0;
  public double MaxAngularSpeedRadians = Math.PI;
  public double LowGearScalar = 0.5;
  public boolean StartInHighGear = false;

  public double[] DrivePID = new double[] { 0, 0, 0 };
  public double[] SteeringPID = new double[] { 0, 0, 0 };
  public double[] SnapToPID = new double[] { 0, 0, 0 };

  public double DriveDeadband = 0;
  public double DeadbandCurveWeight = 0;

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
    double[] drivePID,
    double[] steeringPID,
    double[] snapToPID,
    double driveDeadband,
    double deadbandCurveWeight
  ) {
    TrackWidthMeters = trackWidthMeters;
    WheelBaseMeters = wheelBaseMeters;
    WheelBaseCircumferenceMeters = wheelBaseCircumferenceMeters;

    PigeonId = pigeonId;

    MaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    MaxAccelerationMetersPerSecondSquared =
      maxAccelerationMetersPerSecondSquared;
    MaxAngularSpeedRadians = maxAngularSpeedRadians;
    LowGearScalar = lowGearScalar;
    StartInHighGear = startInHighGear;

    DrivePID = drivePID;
    SteeringPID = steeringPID;
    SnapToPID = snapToPID;

    DriveDeadband = driveDeadband;
    DeadbandCurveWeight = deadbandCurveWeight;
  }
}
