// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConfig;
import prime.config.PrimeConfigurator;

public class Robot extends TimedRobot {

  private final String m_defaultConfigName = "swerve_test_bot.json";
  private String m_selectedConfigName = m_defaultConfigName;
  private SendableChooser<String> m_configChooser;

  private final String m_defaultAutoName = "default-auto";
  private SendableChooser<String> m_autoChooser;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Set up configuration selection
    m_configChooser = new SendableChooser<String>();
    m_configChooser.setDefaultOption("Default", m_defaultConfigName);
    for (var configName : PrimeConfigurator.getAvailableConfigsInDeploy()) {
      m_configChooser.addOption(configName, configName);
    }

    DriverStation.reportWarning(
      ">> Selected config \"" + m_selectedConfigName + "\"",
      false
    );

    var config = PrimeConfigurator.mapConfigFromJsonFile(
      RobotConfig.class,
      m_defaultConfigName
    );

    m_robotContainer = new RobotContainer(config);

    // Set up autonomous selection
    m_autoChooser = new SendableChooser<String>();
    m_autoChooser.setDefaultOption("Default", m_defaultAutoName);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update the selected config name
    var selectedConfig = m_configChooser.getSelected();
    if (m_selectedConfigName != selectedConfig) {
      // A new config was selected, stop all commands and reconfigure the robot
      CommandScheduler.getInstance().cancelAll();
      DriverStation.reportWarning(
        ">> Switching to new config \"" + selectedConfig + "\"",
        false
      );

      m_selectedConfigName = selectedConfig;
      var config = PrimeConfigurator.mapConfigFromJsonFile(
        RobotConfig.class,
        m_selectedConfigName
      );

      m_robotContainer.reconfigure(config);
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
