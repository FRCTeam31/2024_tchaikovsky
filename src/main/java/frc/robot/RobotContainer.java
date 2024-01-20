// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Drivetrain;
import prime.config.Controls;

public class RobotContainer {

  public RobotConfig m_config;
  public Drivetrain Drivetrain;
  public PowerDistribution Pdp;
  public CommandJoystick DriverController;

  public RobotContainer(RobotConfig config) {
    m_config = config;
    Pdp = new PowerDistribution();
    Pdp.resetTotalEnergy();
    Pdp.clearStickyFaults();
    SmartDashboard.putData(Pdp);

    Drivetrain = new Drivetrain(m_config);
    Drivetrain.register();
    SmartDashboard.putData(Drivetrain);
    configureBindings();
  }

  private void configureBindings() {
    DriverController = new CommandJoystick(Controls.DRIVER_PORT);
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
        () -> DriverController.getRawAxis(Controls.RIGHT_STICK_Y),
        () -> DriverController.getRawAxis(Controls.RIGHT_STICK_X),
        () -> DriverController.getRawAxis(Controls.LEFT_STICK_X),
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
