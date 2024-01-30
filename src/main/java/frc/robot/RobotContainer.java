// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    AutoBuilder.configureHolonomic(
      Drivetrain::getPose, // Robot pose supplier
      Drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      Drivetrain::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(0, 0, 0), // Translation PID constants
        new PIDConstants(0, 0, 0), // Rotation PID constants
        m_config.Drivetrain.MaxSpeedMetersPerSecond, // Max module speed, in m/s
        m_config.Drivetrain.WheelBaseCircumferenceMeters / Math.PI / 2, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      Drivetrain // Reference to this subsystem to set requirements
    );
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("line 1m");
    Drivetrain.resetGyro();

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
