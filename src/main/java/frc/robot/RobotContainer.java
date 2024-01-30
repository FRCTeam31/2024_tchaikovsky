// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Drivetrain;
import prime.config.Controls;

public class RobotContainer {

  public RobotConfig m_config;
  public Drivetrain Drivetrain;
  public CommandXboxController DriverController;

  public RobotContainer(RobotConfig config) {
    m_config = config;

    Drivetrain = new Drivetrain(m_config);
    SmartDashboard.putData(Drivetrain);
    configureBindings();
  }

  private void configureBindings() {
    DriverController = new CommandXboxController(Controls.DRIVER_PORT);
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
        () -> DriverController.getRawAxis(Controls.LEFT_STICK_Y),
        () -> DriverController.getRawAxis(Controls.LEFT_STICK_X),
        () ->
          DriverController.getRawAxis(Controls.RIGHT_TRIGGER) -
          DriverController.getRawAxis(Controls.LEFT_TRIGGER),
        false
      )
    );

    DriverController
      .pov(Controls.up)
      .onTrue(Drivetrain.driveWithSnapToAngleCommand(Math.toRadians(0)));
    DriverController
      .pov(Controls.right)
      .onTrue(Drivetrain.driveWithSnapToAngleCommand(Math.toRadians(90)));
    DriverController
      .pov(Controls.down)
      .onTrue(Drivetrain.driveWithSnapToAngleCommand(Math.toRadians(180)));
    DriverController
      .pov(Controls.left)
      .onTrue(Drivetrain.driveWithSnapToAngleCommand(Math.toRadians(-90)));

    DriverController.button(Controls.A).onTrue(Drivetrain.resetGyroCommand());
    DriverController
      .button(Controls.B)
      .onTrue(Drivetrain.toggleShifterCommand());
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    Drivetrain.resetGyro();

    // Sets the starting Position on the path.
    // Rotation2d startingRotation = new Rotation2d(0);
    // Pose2d startingPosition = new Pose2d(1, 5.23, startingRotation);
    // Drivetrain.m_field.setRobotPose(startingPosition);

    PathPlannerPath path = PathPlannerPath.fromPathFile("line 1m");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
