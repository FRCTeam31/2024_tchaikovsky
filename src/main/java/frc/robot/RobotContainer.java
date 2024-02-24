// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  public RobotConfig m_config;
  public ShuffleboardTab d_robotTab = Shuffleboard.getTab("Robot");

  public PrimeXboxController m_driverController;
  public PrimeXboxController m_operatorController;
  public Drivetrain m_drivetrain;
  public Shooter m_shooter;
  public Intake m_intake;
  public Climbers m_climbers;
  public Limelight m_limelight;
  public LEDStrips m_leds;
  public PowerDistribution m_pdh;

  public RobotContainer(RobotConfig config) {
    reconfigure(config);
  }

  /**
   * Stops all commands and reconfigures the robot with a new config instance
   * @param config
   */
  public void reconfigure(RobotConfig config) {
    try {
      // Stop all commands
      CommandScheduler.getInstance().cancelAll();

      // Close subsystems before reconfiguring
      if (m_drivetrain != null) m_drivetrain.close();
      if (m_shooter != null) m_shooter.close();
      if (m_intake != null) m_intake.close();
      if (m_climbers != null) m_climbers.close();
      if (m_leds != null) m_leds.close();
      if (m_pdh != null) m_pdh.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      m_drivetrain = new Drivetrain(m_config);
      m_shooter = new Shooter(m_config.Shooter);
      m_intake = new Intake(m_config.Intake);
      m_climbers = new Climbers(m_config.Climbers);
      m_limelight = new Limelight(m_config.LimelightPose);
      m_leds = new LEDStrips(m_config.LEDs);
      m_pdh = new PowerDistribution();

      // Reconfigure bindings
      configureTeleopControls();
      // Register the named commands from each subsystem that may be used in PathPlanner
      // NamedCommands.registerCommands(m_drivetrain.getNamedCommands());
      // NamedCommands.registerCommands(m_shooter.getNamedCommands());
      // NamedCommands.registerCommands(m_intake.getNamedCommands());
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to configure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }
  }

  public void configureRobotDashboard() {
    d_robotTab.add("Power Hub", m_pdh);
  }

  /**
   * Creates the controller and configures teleop controls
   */
  public void configureTeleopControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
    m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

    // Controls for Driving
    m_drivetrain.setDefaultCommand(
      m_drivetrain.defaultDriveCommand(
        m_driverController.getRightStickYSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getRightStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getLeftStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        true
      )
    );

    // Controls for Snap-To
    m_driverController
      .pov(Controls.up)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(0)));
    m_driverController
      .pov(Controls.left)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(90)));
    m_driverController
      .pov(Controls.down)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(180)));
    m_driverController
      .pov(Controls.right)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(270)));

    m_driverController
      .button(Controls.A)
      .onTrue(m_drivetrain.resetGyroCommand());
    m_driverController
      .button(Controls.B)
      .onTrue(m_drivetrain.toggleShifterCommand());

    // Climbers
    m_driverController.y().onTrue(m_climbers.toggleClimbControlsCommand());
    m_climbers.setDefaultCommand(
      m_climbers.defaultClimbingCommand(
        m_driverController.button(Controls.RB),
        m_driverController.button(Controls.LB),
        () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
        () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
      )
    );
    // Operator Controls =================================

    // Always be updating the intake angle PID
    // m_intake.setDefaultCommand(m_intake.seekAngleSetpointCommand());

    m_operatorController.a().onTrue(m_intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out
    // m_operatorController // Raise intake, load note for amp score
    //   .y()
    //   .onTrue(
    //     m_intake
    //       .setIntakeInCommand()
    //       .andThen(m_intake.waitForIntakeToReachAngleSetpointCommand())
    //       .andThen(m_shooter.loadNoteForAmp())
    //   );
    // m_operatorController // Raise intake, unload note from shooter into intake
    //   .b()
    //   .onTrue(
    //     m_intake
    //       .setIntakeInCommand()
    //       .andThen(m_intake.waitForIntakeToReachAngleSetpointCommand())
    //   )
    //   .whileTrue(
    //     m_shooter
    //       .unloadNoteForSpeaker()
    //       .alongWith(m_intake.setRollersSpeedCommand(() -> 0.5))
    //   )
    //   .onFalse(
    //     m_shooter.stopMotorsCommand().alongWith(m_intake.stopRollersCommand())
    //   );

    m_operatorController // Raise shooter, score in amp
      .rightBumper()
      .onTrue(m_shooter.setElevationUpCommand()) // wait is integrated
      .whileTrue(m_shooter.scoreInAmp())
      .onFalse(
        m_shooter
          .stopMotorsCommand()
          .andThen(m_shooter.setElevationDownCommand()) // wait is integrated
      );

    // m_operatorController // Shoot into Speaker with both intake and shooter
    //   .leftBumper()
    //   .onTrue(m_shooter.setElevationDownCommand()) // wait is integrated
    //   .whileTrue(
    //     m_intake
    //       .setRollersSpeedCommand(() -> -1) // eject the note at full speed
    //       .alongWith(m_shooter.scoreInSpeaker()) // score the note in the speaker
    //   )
    //   .onFalse(
    //     m_intake.stopRollersCommand().alongWith(m_shooter.stopMotorsCommand()) // stop everything
    //   );

    m_operatorController // intake note
      .leftTrigger(0.1)
      .whileTrue(
        m_intake.setRollersSpeedCommand(() ->
          m_operatorController.getLeftTriggerAxis()
        )
      )
      .onFalse(m_intake.stopRollersCommand());

    m_operatorController // eject note
      .rightTrigger(0.1)
      .whileTrue(m_intake.ejectNoteCommand())
      .onFalse(m_intake.stopRollersCommand());
  }

  /**
   * Configures the controllers and binds test & SysID commands to buttons
   */
  public void configureTestControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

    m_driverController
      .start()
      .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .back()
      .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .rightBumper()
      .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .leftBumper()
      .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));
  }
}
