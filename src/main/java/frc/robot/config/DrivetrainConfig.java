package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;

public class DrivetrainConfig {
    public double TrackWidthMeters = 0;
    public double WheelBaseMeters = 0;
    public double WheelBaseCircumferenceMeters = 0;

    public int PigeonId = 0;

    public int MaxSpeedMetersPerSecond = 0;
    public int MaxAccelerationMetersPerSecondSquared = 0;
    public double MaxAngularSpeedRadians = Math.PI;
    public double LowGearScalar = 0.5;

    public PIDConstants SnapToPidConstants = new PIDConstants(0, 0, 0);

    public DrivetrainConfig(double trackWidthMeters,
                            double wheelBaseMeters,
                            double wheelBaseCircumferenceMeters,
                            int pigeonId,
                            int maxSpeedMetersPerSecond,
                            int maxAccelerationMetersPerSecondSquared,
                            double maxAngularSpeedRadians,
                            double lowGearScalar,
                            PIDConstants snapToPidConstants) {
        TrackWidthMeters = trackWidthMeters;
        WheelBaseMeters = wheelBaseMeters;
        WheelBaseCircumferenceMeters = wheelBaseCircumferenceMeters;
        
        PigeonId = pigeonId;
        
        MaxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        MaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
        MaxAngularSpeedRadians = maxAngularSpeedRadians;
        LowGearScalar = lowGearScalar;

        SnapToPidConstants = snapToPidConstants;
    }
}
