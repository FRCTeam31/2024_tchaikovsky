// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import prime.config.PrimeConfigurator;

public class Robot extends TimedRobot {

  private final String m_defaultConfigName = "swerve_test_bot.json";
  private String m_selectedConfigName = m_defaultConfigName;
  private RobotContainer m_robotContainer;

  private SendableChooser<String> m_configChooser;
  private SendableChooser<Command> m_autoChooser;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    // Create the robot container with the default config
    m_robotContainer =
      new RobotContainer(readConfigFromFile(m_defaultConfigName));

    // Set up configuration selection
    m_configChooser = new SendableChooser<String>();
    m_configChooser.setDefaultOption("Default", m_defaultConfigName);
    for (var configName : PrimeConfigurator.getAvailableConfigsInDeploy()) m_configChooser.addOption(
      configName,
      configName
    );
    SmartDashboard.putData("Config Chooser", m_configChooser);

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser("1m Auto");
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Check the selected config name to see if it has changed
    if (m_selectedConfigName != m_configChooser.getSelected()) {
      // A new config has been selected, save the new name and reconfigure the robot
      m_selectedConfigName = m_configChooser.getSelected();
      configureRobot(m_selectedConfigName);
    }
  }

  @Override
  public void autonomousInit() {
    // ENABLE THIS CODE TO USE THE AUTO CHOOSER
    m_autonomousCommand = m_autoChooser.getSelected();

    // // ENABLE THIS CODE TO USE A SPECIFIC AUTO COMMAND
    // m_autonomousCommand = new PathPlannerAuto("1m Auto");

    if (m_autonomousCommand == null || m_autonomousCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      return;
    }

    // Load the path you want to follow using its name in the GUI
    m_robotContainer.m_drivetrain.resetGyro();

    // Reset the starting position on the path.
    m_robotContainer.m_drivetrain.m_field.setRobotPose(
      new Pose2d(0, 0, new Rotation2d(0))
    );

    m_autonomousCommand.schedule();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.configureTeleopControls();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    // Configure test controls
  }

  private void configureRobot(String newConfigName) {
    DriverStation.reportWarning(
      ">> Configuring robot to \"" + newConfigName + "\" profile",
      false
    );

    // Read the config from the file and reconfigure the robot
    var config = readConfigFromFile(newConfigName);
    m_robotContainer.reconfigure(config);
  }

  private RobotConfig readConfigFromFile(String fileName) {
    var config = PrimeConfigurator.mapConfigFromJsonFile(
      RobotConfig.class,
      fileName
    );

    if (config == null) {
      DriverStation.reportError(
        "[ERROR] >> Failed to read config file \"" + fileName + "\"",
        false
      );

      return null;
    }

    return config;
  }
}
