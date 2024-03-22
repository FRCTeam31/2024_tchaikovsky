// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.gyroIO;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSim implements IGyro {

  private double yaw = Math.PI / 4;

  @Override
  public void updateInputs(GyroIOInputs inputs, double angularVelocity) {
    yaw += angularVelocity * 0.02;
    inputs.Rotation2dCCW = new Rotation2d(yaw);
  }

  @Override
  public void setYaw(double yaw) {
    this.yaw = yaw;
  }
}
