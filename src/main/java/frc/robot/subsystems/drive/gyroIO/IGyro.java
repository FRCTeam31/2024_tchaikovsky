// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.gyroIO;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * climber subsystem hardware interface.
 */
public interface IGyro {
  /**
   * Contains all of the input data received from hardware.
   */
  @AutoLog
  public static class GyroIOInputs {

    public Rotation2d Rotation2dCCW = new Rotation2d(0);
  }

  /**
   * Updates the set of loggable inputs.
   */
  public default void updateInputs(GyroIOInputs inputs, double angularVelocity) {}

  public void setYaw(double yaw);
}
