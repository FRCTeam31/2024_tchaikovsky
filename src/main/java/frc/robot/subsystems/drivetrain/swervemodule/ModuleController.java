package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem.DriveMap;

public class ModuleController {

  private SwerveModuleSubsystem m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;

  /**
   * Creates a new SwerveController with the specified configuration and PID constants.
   * @param config
   * @param drivePID
   * @param steeringPID
   */
  public ModuleController(boolean isReal) {
    // Create swerve modules in CCW order from FL to FR
    m_frontLeftModule = new SwerveModuleSubsystem(isReal, DriveMap.FrontLeftSwerveModule);
    m_frontRightModule = new SwerveModuleSubsystem(isReal, DriveMap.FrontRightSwerveModule);
    m_rearLeftModule = new SwerveModuleSubsystem(isReal, DriveMap.RearLeftSwerveModule);
    m_rearRightModule = new SwerveModuleSubsystem(isReal, DriveMap.RearRightSwerveModule);
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * @param desiredStates
   */
  public void setDesiredStates(SwerveModuleState[] desiredStates) {
    m_frontLeftModule.setDesiredState(desiredStates[0]);
    m_frontRightModule.setDesiredState(desiredStates[1]);
    m_rearLeftModule.setDesiredState(desiredStates[2]);
    m_rearRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the instantaneous states for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeftModule.getModuleState(),
      m_frontRightModule.getModuleState(),
      m_rearLeftModule.getModuleState(),
      m_rearRightModule.getModuleState(),
    };
  }

  /**
   * Gets the cumulative positions for each swerve module in FL, FR, RL, RR order
   */
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
    };
  }

  /**
   * Stops all module motors
   */
  public void stopAllMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }
}
