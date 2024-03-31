package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.config.RobotConfig;
import prime.control.PrimePIDConstants;

public class SwerveController {

  private SwerveModule m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;

  // Logging
  private StructArrayPublisher<SwerveModuleState> m_desiredModulesStatesPublisher;
  private StructArrayPublisher<SwerveModuleState> m_measuredModulesStatesPublisher;
  private StructArrayPublisher<SwerveModulePosition> m_measuredModulesPositionsPublisher;

  /**
   * Creates a new SwerveController with the specified configuration and PID constants.
   * @param config
   * @param drivePID
   * @param steeringPID
   */
  public SwerveController(RobotConfig config, PrimePIDConstants drivePID, PrimePIDConstants steeringPID) {
    // Create swerve modules in CCW order from FL to FR
    m_frontLeftModule = new SwerveModule(config.FrontLeftSwerveModule, drivePID, steeringPID);
    m_frontRightModule = new SwerveModule(config.FrontRightSwerveModule, drivePID, steeringPID);
    m_rearLeftModule = new SwerveModule(config.RearLeftSwerveModule, drivePID, steeringPID);
    m_rearRightModule = new SwerveModule(config.RearRightSwerveModule, drivePID, steeringPID);

    // Start logging modules states
    m_desiredModulesStatesPublisher =
      NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("Drive/DesiredSwerveModuleStates", SwerveModuleState.struct)
        .publish();
    m_measuredModulesStatesPublisher =
      NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("Drive/MeasuredSwerveModuleStates", SwerveModuleState.struct)
        .publish();
    m_measuredModulesPositionsPublisher =
      NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("Drive/MeasuredSwerveModulePositions", SwerveModulePosition.struct)
        .publish();
  }

  /**
   * Sets the desired states for each swerve module in order FL, FR, RL, RR
   * @param desiredStates
   */
  public void setDesiredStates(SwerveModuleState[] desiredStates) {
    // Log desired states
    m_desiredModulesStatesPublisher.set(desiredStates);

    m_frontLeftModule.setDesiredState(desiredStates[0]);
    m_frontRightModule.setDesiredState(desiredStates[1]);
    m_rearLeftModule.setDesiredState(desiredStates[2]);
    m_rearRightModule.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    var states = new SwerveModuleState[] {
      m_frontLeftModule.getModuleState(),
      m_frontRightModule.getModuleState(),
      m_rearLeftModule.getModuleState(),
      m_rearRightModule.getModuleState(),
    };

    // Log measured states
    m_measuredModulesStatesPublisher.set(states);

    return states;
  }

  public SwerveModulePosition[] getPositions() {
    var positions = new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
    };

    // Log measured positions
    m_measuredModulesPositionsPublisher.set(positions);

    return positions;
  }

  public void stopAllMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }
}
