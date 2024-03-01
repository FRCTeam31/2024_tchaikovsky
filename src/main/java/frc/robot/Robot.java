// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import java.util.Map;
import prime.config.PrimeConfigurator;
import prime.control.LEDs.Color;
import prime.control.LEDs.SectionState;

public class Robot extends TimedRobot {

  private ShuffleboardTab d_robotTab = Shuffleboard.getTab("Robot");
  private GenericEntry d_allianceEntry = d_robotTab
    .add("Alliance Color", false)
    .withSize(3, 0)
    .withPosition(12, 0)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(
      Map.of("Color when true", "#FF0000", "Color when false", "#0000FF")
    )
    .getEntry();

  public boolean autoEnabled = false;

  private final String m_defaultAutoName = "Speaker Auto 1";
  public static SendableChooser<Command> m_autoChooser;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(RobotConfig.getDefault());
    // m_robotContainer.LEDs.setLeftSection(
    //   0,
    //   SectionState.solidColor(Color.GREEN)
    // );
    // m_robotContainer.LEDs.setRightSection(
    //   0,
    //   SectionState.solidColor(Color.GREEN)
    // );

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser(m_defaultAutoName);
    d_robotTab
      .add(m_autoChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(3, 1)
      .withPosition(3, 3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    d_allianceEntry.setBoolean(
      DriverStation.getAlliance().get() == Alliance.Red
    );
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    autoEnabled = true;

    Pose2d startingPose;
    if (m_autoChooser != null) {
      m_autonomousCommand = m_autoChooser.getSelected();
      startingPose =
        PathPlannerAuto.getStaringPoseFromAutoFile(
          m_autonomousCommand.getName()
        );
    } else {
      m_autonomousCommand = new PathPlannerAuto(m_defaultAutoName);
      startingPose = new Pose2d(1.4, 5.5, Rotation2d.fromDegrees(270));
    }

    // Exit without scheduling an auto command if none is selected
    if (m_autonomousCommand == null || m_autonomousCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      return;
    }

    // m_robotContainer.LEDs.setLeftSection(0, SectionState.solidColor(Color.RED));
    // m_robotContainer.LEDs.setRightSection(
    //   0,
    //   SectionState.solidColor(Color.RED)
    // );

    m_robotContainer.Drivetrain.resetGyro();
    m_robotContainer.Drivetrain.resetOdometry(startingPose);
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

    m_robotContainer.Drivetrain.resetGyro();
    m_robotContainer.Drivetrain.resetOdometry(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );

    // m_robotContainer.LEDs.setLeftSection(
    //   0,
    //   SectionState.solidColor(Color.BLUE)
    // );
    // m_robotContainer.LEDs.setRightSection(
    //   0,
    //   SectionState.solidColor(Color.BLUE)
    // );

    m_robotContainer.configureTeleopControls();
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.Drivetrain.resetGyro();
    m_robotContainer.Drivetrain.resetOdometry(
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
