package frc.robot.config;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfig {

  public String ModuleName = "DefaultModuleName";

  public int DriveMotorCanId = 0;
  public int SteeringMotorCanId = 0;
  public int CANCoderCanId = 0;

  public int StartingOffset = 0;
  public boolean DriveInverted = false;

  public PIDConstants DrivePidConstants = new PIDConstants(0);
  public PIDConstants SteeringPidConstants = new PIDConstants(0);

  public Translation2d ModuleLocation = new Translation2d();

  public SwerveModuleConfig(
    String moduleName,
    int driveMotorCanId,
    int steeringMotorCanId,
    int canCoderCanId,
    int startingOffset,
    boolean driveInverted,
    PIDConstants drivePid,
    PIDConstants steerPid,
    Translation2d location
  ) {
    ModuleName = moduleName;
  }
}
