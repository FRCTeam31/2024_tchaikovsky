package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private boolean m_shooterIsUp;

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
    m_victorSPX = new VictorSPX(m_config.VictorSPXCanID);
    m_victorSPX.configFactoryDefault();

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
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    m_talonFX.set(speed);
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
    return m_noteDetector.get();
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

    if (currentPosition < setpoint) {
      // if the current position is below the setpoint, run the actuator forward
      m_rightLinearActuator.runForward();
    } else if (currentPosition > setpoint) {
      // if the current position is above the setpoint, run the actuator in reverse
      m_rightLinearActuator.runReverse();
    } else {
      // if the current position is at the setpoint, stop the actuator
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

  /**
   * Stops the shooter motors
   * @return
   */
  public Command stopMotorsCommand() {
    return this.run(() -> stopMotors());
  }

  /**
   * Shootes a note at half speed
   * @return
   */
  public Command scoreInAmp() {
    return this.run(() -> runShooter(0.5));
  }

  /**
   * Shootes a note at full speed
   * @return
   */
  public Command scoreInSpeaker() {
    return this.run(() -> runShooter(1));
  }

  /**
   * Sets the elevation of the shooter all the way up
   * @return
   */
  public Command setElevationUpCommand() {
    return this.runOnce(() -> m_shooterIsUp = true);
  }

  /**
   * Sets the elevation of the shooter all the way down
   * @return
   */
  public Command setElevationDownCommand() {
    return this.runOnce(() -> m_shooterIsUp = false);
  }

  /**
   * Toggles the elevation of the shooter up/down
   * @return
   */
  public Command toggleElevationCommand() {
    return this.runOnce(() -> m_shooterIsUp = !m_shooterIsUp);
  }

  /**
   * Constantly seeks the elevation setpoint until cancelled
   * @return
   */
  public Command seekElevationSetpointCommand() {
    return this.run(() -> seekElevationSetpoint());
  }

  /**
   * Waits for the elevation to reach the setpoint
   * @return
   */
  public Command waitForElevationToReachSetpointCommand() {
    return this.runOnce(() -> {
        var setpoint = m_shooterIsUp
          ? m_config.MaximumElevation
          : m_config.MinimumElevation;

        var delta = Math.abs(getRightActuatorPosition() - setpoint);

        while (delta > 0.05) {
          // wait for the actuator to reach the setpoint
        }
      });
  }

  /**
   * Loads a note into the shooter for dropping into the amp
   */
  public Command loadNoteForAmp() {
    return this.runOnce(() -> {
        while (!isNoteLoaded()) {
          runShooter(0.5);
        }
        stopMotors();
      });
  }

  /**
   * Unloads a note from the shooter for shooting into the Speaker
   */
  public Command unloadNoteForSpeaker() {
    return this.runOnce(() -> {
        while (!isNoteLoaded()) {
          runShooter(0.5);
        }
        stopMotors();
      });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      // "Example_Command", exampleCommand(),
      "Stop_Shooter_Motors",
      stopMotorsCommand(),
      "Score_In_Amp",
      scoreInAmp(),
      "Score_In_Speaker",
      scoreInSpeaker(),
      "Set_Actuators_Up",
      setElevationUpCommand(),
      "Set_Actuators_Down",
      setElevationDownCommand(),
      "Load_Amp",
      loadNoteForAmp(),
      "Unload_Shooter",
      unloadNoteForSpeaker()
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
