// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import prime.config.PrimeConfigurator;

public class Robot extends TimedRobot {

  public ShuffleboardTab d_robotTab = Shuffleboard.getTab("Robot");
  public boolean autoEnabled = false;

  private SendableChooser<String> m_configChooser;
  private final String m_defaultConfigName = "swerve_test_bot.json";
  private String m_selectedConfigName = m_defaultConfigName;

  public static SendableChooser<Command> m_autoChooser;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Create the robot container with the default config
    // m_robotContainer =
    //   new RobotContainer(readConfigFromFile(m_defaultConfigName));
    m_robotContainer = new RobotContainer(RobotConfig.getDefault());

    // Set up configuration selection
    m_configChooser = PrimeConfigurator.buildConfigChooser(m_defaultConfigName);
    d_robotTab
      .add(m_configChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 1)
      .withPosition(0, 0);

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    d_robotTab
      .add(m_autoChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 1)
      .withPosition(0, 1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
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

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    autoEnabled = true;

    m_robotContainer.m_drivetrain.resetGyro();
    m_robotContainer.m_drivetrain.resetOdometry(
      new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(270))
    );

    // ENABLE THIS CODE TO USE THE AUTO CHOOSER
    if (m_autoChooser != null) {
      m_autonomousCommand = m_autoChooser.getSelected();
    } else {
      m_autonomousCommand = new PathPlannerAuto("line 1m");
    }

    // Exit without scheduling an auto command if none is selected
    if (m_autonomousCommand == null || m_autonomousCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      return;
    }

    m_autonomousCommand.schedule();
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_drivetrain.resetGyro();
    m_robotContainer.m_drivetrain.resetOdometry(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );
    m_robotContainer.configureTeleopControls();
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.m_drivetrain.resetGyro();
    m_robotContainer.m_drivetrain.resetOdometry(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );
    m_robotContainer.configureTestControls();

    // Start the data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * Configures the robot with a new config file
   * @param newConfigName The name of the new config file
   */
  private void configureRobot(String newConfigName) {
    DriverStation.reportWarning(
      ">> Configuring robot to \"" + newConfigName + "\" profile",
      false
    );

    // Read the config from the file and reconfigure the robot
    var config = readConfigFromFile(newConfigName);
    m_robotContainer.reconfigure(config);
  }

  /**
   * Reads a RobotConfig from a JSON file
   * @param fileName The name of the JSON file to read
   * @return A RobotConfig instance or null if the file could not be read
   */
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
