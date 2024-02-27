package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ShooterConfig;
import java.util.Map;
import prime.movers.IPlannable;

public class Shooter extends SubsystemBase implements IPlannable {

  private ShooterConfig m_config;

  private TalonFX m_talonFX;
  private VictorSPX m_victorSPX;
  private DigitalInput m_noteDetector;
  private DoubleSolenoid m_leftDoubleSolenoid;
  private DoubleSolenoid m_rightDoubleSolenoid;

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

    m_noteDetector = new DigitalInput(m_config.NoteDetectorDIOChannel);
    m_shooterIsUp = false;

    m_leftDoubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    m_rightDoubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
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
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return
   */
  public boolean isNoteLoaded() {
    return m_noteDetector.get();
  }

  public void extendShooter() {
    m_leftDoubleSolenoid.set(Value.kForward);
    m_rightDoubleSolenoid.set(Value.kForward);
  }

  public void retractShooter() {
    m_leftDoubleSolenoid.set(Value.kReverse);
    m_rightDoubleSolenoid.set(Value.kReverse);
  }

  //#endregion

  @Override
  public void periodic() {
    d_talonFXSpeed.setDouble(m_talonFX.get());
    d_victorSPXSpeed.setDouble(m_victorSPX.getMotorOutputPercent());
    d_noteDetector.setBoolean(isNoteLoaded());
    d_shooterIsUp.setBoolean(m_shooterIsUp);
  }

  //#region Shooter Commands

  public Command runShooterForTime(long timeInMilliseconds, int speed) {
    return this.run(() -> {
        long initTime = RobotController.getFPGATime();
        while (RobotController.getFPGATime() - initTime <= timeInMilliseconds) {
          runShooter(speed);
        }
      });
  }

  /**
   * Stops the shooter motors
   * @return
   */
  public Command stopMotorsCommand() {
    return this.run(() -> stopMotors());
  }

  /**
   * Shootes a note at full speed
   * @return
   */
  public Command runShooterCommand() {
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

  public Command togglePneumaticsCommand() {
    return this.runOnce(() -> {
        if (m_leftDoubleSolenoid.get() == Value.kForward) {
          retractShooter();
        } else if (m_leftDoubleSolenoid.get() == Value.kReverse) {
          extendShooter();
        }
      });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      // "Example_Command", exampleCommand(),
      "Stop_Shooter_Motors",
      stopMotorsCommand(),
      "Run_Shooter",
      runShooterCommand(),
      "Set_Actuators_Up",
      setElevationUpCommand(),
      "Set_Actuators_Down",
      setElevationDownCommand(),
      "Load_Amp",
      loadNoteForAmp(),
      "Unload_Shooter",
      unloadNoteForSpeaker(),
      "Run_Shooter_For_2_Seconds",
      runShooterForTime(2000, 1)
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
