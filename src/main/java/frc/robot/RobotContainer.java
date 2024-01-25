// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Drivetrain;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  public RobotConfig m_config;
  public Drivetrain Drivetrain;
  public PrimeXboxController DriverController;

  public RobotContainer(RobotConfig config) {
    m_config = config;

    Drivetrain = new Drivetrain(m_config);
    DriverController = new PrimeXboxController(Controls.DRIVER_PORT);

    configureBindings();
  }

  public void reconfigure(RobotConfig config) {
    try {
      // Close subsystems before reconfiguring
      Drivetrain.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      Drivetrain = new Drivetrain(m_config);

      // Reconfigure bindings
      configureBindings();
    } catch (Exception e) {
      DriverStation.reportError(
        "Failed to reconfigure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }
  }

  private void configureBindings() {
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
        DriverController.getLeftStickYSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        DriverController.getLeftStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        DriverController.getTriggerSupplier(),
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
    return Commands.print("No autonomous command configured");
  }
}
