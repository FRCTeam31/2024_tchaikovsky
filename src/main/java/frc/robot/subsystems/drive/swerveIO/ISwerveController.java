// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerveIO;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * climber subsystem hardware interface.
 */
public abstract class ISwerveController {

  protected ISwerveModule frontLeftModule;
  protected ISwerveModule frontRightModule;
  protected ISwerveModule rearLeftModule;
  protected ISwerveModule rearRightModule;

  /**
   * Contains all of the input data received from hardware.
   */
  @AutoLog
  public static class ISwerveControllerInputs {

    public SwerveModuleState[] DesiredStates = new SwerveModuleState[4];
    public SwerveModuleState[] MeasuredStates = new SwerveModuleState[4];
    public SwerveModulePosition[] Positions = new SwerveModulePosition[4];
  }

  /**
   * Updates the set of loggable inputs.
   */
  public void updateInputs(ISwerveControllerInputs inputs) {
    inputs.DesiredStates = getModuleStates();
    inputs.MeasuredStates = getModuleStates();
    inputs.Positions = getModulePositions();
  }

  /**
   * Sets the desired states of the swerve modules using an array in order FL, FR, RL, RR
   * @param states
   */
  public void setDesiredStates(SwerveModuleState[] states) {
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    rearLeftModule.setDesiredState(states[2]);
    rearRightModule.setDesiredState(states[3]);
  }

  /**
   * Gets the module states as an array in order FL, FR, RL, RR
   * @return
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getModuleState(),
      frontRightModule.getModuleState(),
      rearLeftModule.getModuleState(),
      rearRightModule.getModuleState(),
    };
  }

  /**
   * Gets the module positions as an array in order FL, FR, RL, RR
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getModulePosition(),
      frontRightModule.getModulePosition(),
      rearLeftModule.getModulePosition(),
      rearRightModule.getModulePosition(),
    };
  }

  /**
   * Stops all swerve module motors
   */
  public void stopMotors() {
    frontLeftModule.stopMotors();
    frontRightModule.stopMotors();
    rearLeftModule.stopMotors();
    rearRightModule.stopMotors();
  }
}
