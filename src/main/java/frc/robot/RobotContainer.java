// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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

  private RobotConfig m_config;
  private ShuffleboardTab d_robotTab = Shuffleboard.getTab("Robot");
  private PrimeXboxController m_driverController;
  private PrimeXboxController m_operatorController;

  public Drivetrain Drivetrain;
  public Shooter Shooter;
  public Intake Intake;
  public Climbers Climbers;
  public Limelight Limelight;
  public LEDStrips LEDs;
  public PowerDistribution PDH;

  public RobotContainer(RobotConfig config) {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
    m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

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
      if (Drivetrain != null) Drivetrain.close();
      if (Shooter != null) Shooter.close();
      if (Intake != null) Intake.close();
      if (Climbers != null) Climbers.close();
      if (LEDs != null) LEDs.close();
      if (PDH != null) PDH.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      Drivetrain = new Drivetrain(m_config);
      Shooter = new Shooter(m_config.Shooter);
      Intake = new Intake(m_config.Intake);
      Climbers = new Climbers(m_config.Climbers);
      Limelight = new Limelight(m_config.LimelightPose);
      // LEDs = new LEDStrips(m_config.LEDs);
      PDH = new PowerDistribution();

      // Reconfigure bindings
      configureTeleopControls();

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());
      NamedCommands.registerCommands(Shooter.getNamedCommands());
      NamedCommands.registerCommands(Intake.getNamedCommands());
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to configure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }
  }

  public void configureRobotDashboard() {
    d_robotTab
      .add("Power Hub", PDH)
      .withSize(3, 3)
      .withPosition(3, 0)
      .withWidget(BuiltInWidgets.kPowerDistribution);
  }

  /**
   * Creates the controller and configures teleop controls
   */
  public void configureTeleopControls() {
    // Controls for Driving
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
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
      .onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(0)));
    m_driverController
      .pov(Controls.left)
      .onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(90)));
    m_driverController
      .pov(Controls.down)
      .onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(180)));
    m_driverController
      .pov(Controls.right)
      .onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(270)));

    m_driverController.button(Controls.A).onTrue(Drivetrain.resetGyroCommand());
    m_driverController
      .button(Controls.B)
      .onTrue(Drivetrain.toggleShifterCommand());

    // Climbers
    m_driverController.y().onTrue(Climbers.toggleClimbControlsCommand());
    Climbers.setDefaultCommand(
      Climbers.defaultClimbingCommand(
        m_driverController.button(Controls.RB),
        m_driverController.button(Controls.LB),
        () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
        () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
      )
    );
    // Operator Controls =================================

    // Always be updating the intake angle PID
    Intake.setDefaultCommand(Intake.seekAngleSetpointCommand());
    Shooter.setDefaultCommand(Shooter.seekElevationSetpointCommand());

    m_operatorController.a().onTrue(Intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out // TODO: Test

    m_operatorController // Raise/lower shooter
      .rightBumper()
      .onTrue(Shooter.toggleElevationCommand()); // wait is integrated

    m_operatorController // Shooting the note
      .b()
      .whileTrue(
        Shooter
          .scoreInSpeaker()
          .alongWith(
            Commands.run(
              () -> {
                if (!Shooter.m_shooterIsUp) {
                  Intake.runIntakeRollers(-1);
                }
              },
              Intake
            )
          )
      )
      .onFalse(
        Shooter.stopMotorsCommand().alongWith(Intake.stopRollersCommand())
      );

    m_operatorController // intake note
      .leftTrigger(0.1)
      .whileTrue(
        Intake.setRollersSpeedCommand(() ->
          m_operatorController.getLeftTriggerAxis()
        )
      )
      .onFalse(Intake.stopRollersCommand());

    m_operatorController // eject note
      .rightTrigger(0.1)
      .whileTrue(
        Intake
          .ejectNoteCommand()
          .alongWith(
            Commands.run(
              () -> {
                if (!Shooter.m_shooterIsUp && Intake.m_angleToggledIn) {
                  Shooter.runShooter(0.3);
                }
              },
              Shooter
            )
          )
      )
      .onFalse(
        Intake.stopRollersCommand().alongWith(Shooter.stopMotorsCommand())
      );
  }

  /**
   * Configures the controllers and binds test & SysID commands to buttons
   */
  public void configureTestControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

    m_driverController
      .start()
      .whileTrue(Drivetrain.sysIdQuasistatic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .back()
      .whileTrue(Drivetrain.sysIdQuasistatic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .rightBumper()
      .whileTrue(Drivetrain.sysIdDynamic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .leftBumper()
      .whileTrue(Drivetrain.sysIdDynamic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));
  }
}
