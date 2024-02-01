package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfig {

  public String ModuleName = "DefaultModuleName";

  public int DriveMotorCanId;
  public int SteeringMotorCanId;
  public int CANCoderCanId;

  public double StartingOffset;
  public boolean DriveInverted;
  public boolean SteerInverted;

  public double ModuleLocationXMeters;
  public double ModuleLocationYMeters;

  public double DriveGearRatio;
  public double DriveWheelDiameterMeters;
  public double DriveWheelCircumferenceMeters;

  public ClosedLoopRampsConfigs DriveClosedLoopRampConfiguration = new ClosedLoopRampsConfigs()
    .withTorqueClosedLoopRampPeriod(0.5)
    .withVoltageClosedLoopRampPeriod(0.5)
    .withDutyCycleClosedLoopRampPeriod(0.5);
  public CurrentLimitsConfigs DriveCurrentLimitConfiguration = new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentThreshold(50)
    .withSupplyTimeThreshold(100);

  public SwerveModuleConfig(
    String moduleName,
    int driveMotorCanId,
    int steeringMotorCanId,
    int canCoderCanId,
    double startingOffset,
    boolean driveInverted,
    boolean steerInverted,
    Translation2d location,
    double driveGearRatio,
    double driveWheelDiameterMeters
  ) {
    ModuleName = moduleName;

    DriveMotorCanId = driveMotorCanId;
    SteeringMotorCanId = steeringMotorCanId;
    CANCoderCanId = canCoderCanId;

    StartingOffset = startingOffset;
    DriveInverted = driveInverted;
    SteerInverted = steerInverted;

    ModuleLocationXMeters = location.getX();
    ModuleLocationYMeters = location.getY();

    DriveGearRatio = driveGearRatio;
    DriveWheelDiameterMeters = driveWheelDiameterMeters;
    DriveWheelCircumferenceMeters = Math.PI * DriveWheelDiameterMeters;
  }

  public Translation2d getModuleLocation() {
    return new Translation2d(ModuleLocationXMeters, ModuleLocationYMeters);
  }
}
