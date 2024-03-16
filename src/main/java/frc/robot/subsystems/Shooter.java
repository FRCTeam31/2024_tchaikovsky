package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ShooterConfig;
import java.util.Map;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDSection;
import prime.movers.IPlannable;

public class Shooter extends SubsystemBase implements IPlannable {

  private ShooterConfig m_config;

  private LEDs m_leds;
  private TalonFX m_talonFX;
  private VictorSPX m_victorSPX;
  private DoubleSolenoid m_elevationSolenoid;
  private DigitalInput m_noteDetector;

  // #region Shuffleboard
  // Shuffleboard configuration
  // private ShuffleboardTab d_shooterTab = Shuffleboard.getTab("Shooter");
  // private GenericEntry d_talonFXSpeed = d_shooterTab
  //   .add("TalonFX Speed", 0)
  //   .withWidget(BuiltInWidgets.kDial)
  //   .withProperties(Map.of("Min", -1, "Max", 1))
  //   .getEntry();
  // private GenericEntry d_victorSPXSpeed = d_shooterTab
  //   .add("VictorSPX Speed", 0)
  //   .withWidget(BuiltInWidgets.kDial)
  //   .withProperties(Map.of("Min", -1, "Max", 1))
  //   .getEntry();
  // private GenericEntry d_leftLinearActuator = d_shooterTab
  //   .add("Left Actuator Position", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Min", 0, "Max", 1))
  //   .getEntry();
  // private GenericEntry d_rightLinearActuator = d_shooterTab
  //   .add("Right Actuator Position", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Min", 0, "Max", 1))
  //   .getEntry();
  // private GenericEntry d_noteDetector = d_shooterTab
  //   .add("Note Detected", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .getEntry();
  // private GenericEntry d_shooterIsUp = d_shooterTab
  //   .add("Shooter is up", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .getEntry();
  // private GenericEntry d_pidOutputEntry = d_shooterTab
  //   .add("PID output", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Max", 2, "Min", -2))
  //   .getEntry();

  // #endregion

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public Shooter(ShooterConfig config, LEDs leds) {
    m_config = config;
    m_leds = leds;
    setName("Shooter");

    m_talonFX = new TalonFX(m_config.TalonFXCanID);
    m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
    m_talonFX.setInverted(true);
    m_talonFX.setNeutralMode(NeutralModeValue.Brake);

    m_victorSPX = new VictorSPX(m_config.VictorSPXCanID);
    m_victorSPX.configFactoryDefault();
    m_victorSPX.setNeutralMode(NeutralMode.Brake);

    m_elevationSolenoid =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        m_config.ElevationSolenoidForwardChannel,
        m_config.ElevationSolenoidReverseChannel
      );

    m_noteDetector = new DigitalInput(m_config.NoteDetectorDIOChannel);
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
    m_leds.restoreLastStripState();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return
   */
  public boolean isNoteLoaded() {
    return !m_noteDetector.get();
  }

  public void setElevator(Value value) {
    m_elevationSolenoid.set(value);
  }

  public void setElevatorUp() {
    setElevator(Value.kForward);
    m_leds.setStripTemporary(LEDSection.solidColor(Color.WHITE));
  }

  public void setElevatorDown() {
    setElevator(Value.kReverse);
    m_leds.restoreLastStripState();
  }

  //#endregion

  private boolean m_lastNoteDetectedValue = false;

  @Override
  public void periodic() {
    var newNoteDetectedValue = isNoteLoaded();
    if (newNoteDetectedValue != m_lastNoteDetectedValue) {
      // Save the new value
      m_lastNoteDetectedValue = newNoteDetectedValue;

      if (newNoteDetectedValue) {
        m_leds.setStripTemporary(LEDSection.blinkColor(prime.control.LEDs.Color.ORANGE, 500));
      } else {
        m_leds.restoreLastStripState();
      }
    }
  }

  //#region Shooter Commands

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
  public Command startShootingNoteCommand() {
    return Commands.runOnce(() -> {
      runShooter(1);
      m_leds.setStripTemporary(LEDSection.raceColor(Color.GREEN, 25, isNoteLoaded()));
    });
  }

  /**
   * Sets the elevation of the shooter all the way up
   * @return
   */
  public Command setElevationUpCommand() {
    return Commands.runOnce(this::setElevatorUp);
  }

  /**
   * Sets the elevation of the shooter all the way down
   * @return
   */
  public Command setElevationDownCommand() {
    return Commands.runOnce(this::setElevatorDown);
  }

  /**
   * Toggles the elevation of the shooter up/down
   * @return
   */
  public Command toggleElevationCommand() {
    return Commands.runOnce(() -> {
      if (m_elevationSolenoid.get() == Value.kForward) setElevatorDown(); else setElevatorUp();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of("Set_Elevation_Up", setElevationUpCommand(), "Set_Elevation_Down", setElevationDownCommand());
  }

  //#endregion

  /**
   * Closes the Shooter
   */
  public void close() {
    m_talonFX.close();
    m_victorSPX.DestroyObject();
  }
}
