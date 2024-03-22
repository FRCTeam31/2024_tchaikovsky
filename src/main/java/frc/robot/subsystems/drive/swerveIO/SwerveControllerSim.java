// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swerveIO;

import frc.robot.config.RobotConfig;

public class SwerveControllerSim extends ISwerveController {

  public SwerveControllerSim(RobotConfig config) {
    frontLeftModule = new SwerveModuleSim(config.Drivetrain, config.FrontLeftSwerveModule);
    frontRightModule = new SwerveModuleSim(config.Drivetrain, config.FrontRightSwerveModule);
    rearLeftModule = new SwerveModuleSim(config.Drivetrain, config.RearLeftSwerveModule);
    rearRightModule = new SwerveModuleSim(config.Drivetrain, config.RearRightSwerveModule);
  }
}
