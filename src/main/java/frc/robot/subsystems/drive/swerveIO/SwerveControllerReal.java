package frc.robot.subsystems.drive.swerveIO;

import frc.robot.config.RobotConfig;

public class SwerveControllerReal extends ISwerveController {

  public SwerveControllerReal(RobotConfig config) {
    // Create swerve modules in CCW order from FL to FR
    frontLeftModule =
      new SwerveModuleReal(config.FrontLeftSwerveModule, config.Drivetrain.DrivePID, config.Drivetrain.SteeringPID);
    frontRightModule =
      new SwerveModuleReal(config.FrontRightSwerveModule, config.Drivetrain.DrivePID, config.Drivetrain.SteeringPID);
    rearLeftModule =
      new SwerveModuleReal(config.RearLeftSwerveModule, config.Drivetrain.DrivePID, config.Drivetrain.SteeringPID);
    rearRightModule =
      new SwerveModuleReal(config.RearRightSwerveModule, config.Drivetrain.DrivePID, config.Drivetrain.SteeringPID);
  }
}
