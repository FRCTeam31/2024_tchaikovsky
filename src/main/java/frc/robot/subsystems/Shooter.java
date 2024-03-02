package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.ShooterConfig;
import java.util.Map;
import prime.movers.IPlannable;
import prime.movers.LinearActuator;

public class Shooter extends SubsystemBase implements IPlannable {

  private ShooterConfig m_config;

  private TalonFX m_talonFX;
  private VictorSPX m_victorSPX;
  private LinearActuator m_leftLinearActuator;
  private LinearActuator m_rightLinearActuator;
  private DigitalInput m_noteDetector;

  public boolean m_shooterIsUp;
  private PIDController m_elevationPidController;
  private Debouncer m_elevationToggleDebouncer = new Debouncer(
    0.1,
    Debouncer.DebounceType.kBoth
  );

  // #region Shuffleboard
  // Shuffleboard configuration
  private ShuffleboardTab d_shooterTab = Shuffleboard.getTab("Shooter");
  private GenericEntry d_talonFXSpeed = d_shooterTab
    .add("TalonFX Speed", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Min", -1, "Max", 1))
    .getEntry();
  private GenericEntry d_victorSPXSpeed = d_shooterTab
    .add("VictorSPX Speed", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Min", -1, "Max", 1))
    .getEntry();
  private GenericEntry d_leftLinearActuator = d_shooterTab
    .add("Left Actuator Position", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Min", 0, "Max", 1))
    .getEntry();
  private GenericEntry d_rightLinearActuator = d_shooterTab
    .add("Right Actuator Position", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Min", 0, "Max", 1))
    .getEntry();
  private GenericEntry d_noteDetector = d_shooterTab
    .add("Note Detected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_shooterIsUp = d_shooterTab
    .add("Shooter is up", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_pidOutputEntry = d_shooterTab
    .add("PID output", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Max", 2, "Min", -2))
    .getEntry();

  // #endregion

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public Shooter(ShooterConfig config) {
    m_config = config;
    setName("Shooter");

    m_talonFX = new TalonFX(m_config.TalonFXCanID);
    m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
    m_talonFX.setInverted(true);
    m_talonFX.setNeutralMode(NeutralModeValue.Brake);

    m_victorSPX = new VictorSPX(m_config.VictorSPXCanID);
    m_victorSPX.configFactoryDefault();
    m_victorSPX.setNeutralMode(NeutralMode.Brake);

    m_leftLinearActuator =
      new LinearActuator(
        m_config.LeftLinearActuatorCanID,
        m_config.LeftLinearActuatorAnalogChannel
      );
    m_rightLinearActuator =
      new LinearActuator(
        m_config.RightLinearActuatorCanID,
        m_config.RightLinearActuatorAnalogChannel
      );

    m_noteDetector = new DigitalInput(m_config.NoteDetectorDIOChannel);
    m_shooterIsUp = false;
    m_elevationPidController = new PIDController(25, 0, 0, 0.02);
    m_elevationPidController.setSetpoint(m_config.MinimumElevation);
    d_shooterTab
      .add("Elevation PID", m_elevationPidController)
      .withWidget(BuiltInWidgets.kPIDController);
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    m_talonFX.set(speed);
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed * 3);
  }

  public void runGreenWheel(double speed) {
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /**
   * Stops the shooter motors
   */
  public void stopMotors() {
    m_talonFX.stopMotor();
    m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  /**
   * Gets the position of the right linear actuator
   */
  public double getRightActuatorPosition() {
    return m_rightLinearActuator.getPosition();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return
   */
  public boolean isNoteLoaded() {
    return !m_noteDetector.get();
    // return false;
  }

  /**
   * Sets the elevation of the shooter
   * @param percentRaised How far, in percentage, the shooter should be raised
   */
  public void seekElevationSetpoint() {
    double currentPosition = getRightActuatorPosition();

    // if the shooter is up, set the setpoint to the maximum elevation
    var setpoint = m_shooterIsUp
      ? m_config.MaximumElevation
      : m_config.MinimumElevation;

    var pidOutput = m_elevationPidController.calculate(
      currentPosition,
      setpoint
    );

    d_pidOutputEntry.setDouble(pidOutput);
    var canGoHigher =
      currentPosition < m_config.MaximumElevation && pidOutput > 0;
    var canGoLower =
      currentPosition > m_config.MinimumElevation && pidOutput < 0;

    if (canGoHigher) {
      m_leftLinearActuator.set(pidOutput);
      m_rightLinearActuator.set(pidOutput);
    } else if (canGoLower) {
      m_leftLinearActuator.set(pidOutput);
      m_rightLinearActuator.set(pidOutput);
    } else {
      m_leftLinearActuator.stop();
      m_rightLinearActuator.stop();
    }
  }

  //#endregion

  @Override
  public void periodic() {
    d_talonFXSpeed.setDouble(m_talonFX.get());
    d_victorSPXSpeed.setDouble(m_victorSPX.getMotorOutputPercent());
    d_leftLinearActuator.setDouble(m_leftLinearActuator.getPosition());
    d_rightLinearActuator.setDouble(m_rightLinearActuator.getPosition());
    d_noteDetector.setBoolean(isNoteLoaded());
    d_shooterIsUp.setBoolean(m_shooterIsUp);
  }

  //#region Shooter Commands

  public Command setShooterSpeedCommand() {
    return Commands.run(() -> {});
  }

  /**
   * Stops the shooter motors
   * @return
   */
  public Command stopMotorsCommand() {
    return Commands.runOnce(() -> stopMotors());
  }

  /**
   * Shootes a note at half speed
   * @return
   */
  public Command scoreInAmpCommand() {
    return Commands.run(() -> runShooter(0.5));
  }

  /**
   * Shootes a note at full speed
   * @return
   */
  public Command scoreInSpeakerCommand() {
    return Commands.runOnce(() -> runShooter(1));
  }

  /**
   * Sets the elevation of the shooter all the way up
   * @return
   */
  public Command setElevationUpCommand() {
    return Commands.runOnce(() -> m_shooterIsUp = true);
  }

  /**
   * Sets the elevation of the shooter all the way down
   * @return
   */
  public Command setElevationDownCommand() {
    return Commands.runOnce(() -> m_shooterIsUp = false);
  }

  /**
   * Toggles the elevation of the shooter up/down
   * @return
   */
  public Command toggleElevationCommand() {
    return Commands.runOnce(() -> {
      m_shooterIsUp = !m_elevationToggleDebouncer.calculate(m_shooterIsUp);
    });
  }

  /**
   * Constantly seeks the elevation setpoint until cancelled
   * @return
   */
  public Command seekElevationSetpointCommand() {
    return this.run(() -> seekElevationSetpoint());
  }

  public Command runShooterForTime(double seconds, double speed) {
    return Commands
      .runOnce(() -> {
        runShooter(speed);
      })
      .andThen(new WaitCommand(seconds))
      .andThen(stopMotorsCommand());
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      // "Example_Command", exampleCommand(),
      "Stop_Shooter_Motors",
      stopMotorsCommand(),
      "Score_In_Amp",
      scoreInAmpCommand(),
      "Score_In_Speaker",
      scoreInSpeakerCommand(),
      "Set_Actuators_Up",
      setElevationUpCommand(),
      "Set_Actuators_Down",
      setElevationDownCommand()
    );
  }

  //#endregion

  /**
   * Closes the Shooter
   */
  public void close() {
    m_talonFX.close();
  }
}
