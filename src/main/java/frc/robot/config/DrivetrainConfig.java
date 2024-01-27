package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;

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
  public double DriveBaseRadius = 0;

  public PIDConstants SnapToPidConstants = new PIDConstants(0, 0, 0);

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
    PIDConstants snapToPidConstants
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

    SnapToPidConstants = snapToPidConstants;
  }
}
