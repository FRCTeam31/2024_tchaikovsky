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
import java.util.function.DoubleSupplier;
import prime.movers.LinearActuator;

public class Shooter extends SubsystemBase implements AutoCloseable {

  private ShooterConfig m_config;

  private TalonFX m_talonFX;
  private VictorSPX m_victorSPX;
  private LinearActuator m_leftLinearActuator;
  private LinearActuator m_rightLinearActuator;
  private DigitalInput m_noteDetector;

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
    .add("Note Detected", 0)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public Shooter(ShooterConfig config) {
    m_config = config;
    setName("Shooter");

    m_talonFX = new TalonFX(m_config.TalonFXCanID);
    m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
    m_talonFX.setInverted(m_config.TalonFXInverted);

    m_victorSPX = new VictorSPX(m_config.VictorSPXCanID);
    m_victorSPX.configFactoryDefault();
    m_victorSPX.setInverted(m_config.VictorSPXInverted);

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
   * Raises the shooter until it reaches the top
   */
  public void raiseElevationActuators() {
    if (m_leftLinearActuator.getPosition() <= 0.9) {
      m_leftLinearActuator.runForward();
      m_rightLinearActuator.runForward();
    }
  }

  /**
   * Lowers the shooter until it reaches the bottom
   */
  public void lowerElevationActuators() {
    if (m_leftLinearActuator.getPosition() >= 0.1) {
      m_leftLinearActuator.runReverse();
      m_rightLinearActuator.runReverse();
    }
  }

  /**
   * Stops the elevation actuators
   */
  public void stopElevationActuators() {
    m_leftLinearActuator.stop();
    m_rightLinearActuator.stop();
  }

  //#endregion

  @Override
  public void periodic() {
    d_talonFXSpeed.setDouble(m_talonFX.get());
    d_victorSPXSpeed.setDouble(m_victorSPX.getMotorOutputPercent());
    d_leftLinearActuator.setDouble(m_leftLinearActuator.getPosition());
    d_rightLinearActuator.setDouble(m_rightLinearActuator.getPosition());
    d_noteDetector.setBoolean(m_noteDetector.get());
  }

  //#region Shooter Commands

  /**
   * Runs the shooter motors at a speed
   * @param speed
   * @return
   */
  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> runShooter(speed.getAsDouble()));
  }

  /**
   * Stops the shooter motors
   * @return
   */
  public Command stopMotorsCommand() {
    return this.run(() -> stopMotors());
  }

  /**
   * Raises the elevation actuators
   * @return
   */
  public Command RaiseActuatorsCommand() {
    return this.run(() -> raiseElevationActuators());
  }

  /**
   * Lowers the elevation actuators
   * @return
   */
  public Command LowerActuatorsCommand() {
    return this.run(() -> lowerElevationActuators());
  }

  /**
   * Stops the elevation actuators
   * @return
   */
  public Command stopActuatorsCommand() {
    return this.runOnce(() -> stopElevationActuators());
  }

  //#endregion

  /**
   * Closes the Shooter
   */
  public void close() {
    m_talonFX.close();
  }
}
