package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.Shooter.IShooterIO.ShooterIOInputs;
import frc.robot.subsystems.Shooter.IShooterIO.ShooterIOOutputs;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    public class VMap {
      public static final int TALONFX_CAN_ID = 20;
      public static final int VICTORSPX_CAN_ID = 19;
      public static final boolean TALONFX_INVERTED = false;
      public static final boolean VICTORSPX_INVERTED = false;
      public static final int NOTE_DETECTOR_DIO_CHANNEL = 7;
      public static final int ELEVATION_SOLENOID_FORWARD_CHANNEL = 6;
      public static final int ELEVATION_SOLENOID_REVERSE_CHANNEL = 7;
    }

    private IShooterIO shooterIO;
    private ShooterIOInputs shooterInputs = new ShooterIOInputs();
    private ShooterIOOutputs shooterOutputs = new ShooterIOOutputs();
  // #endregion

  /**
   * Creates a new Shooter with a given configuration
   * @param config
   */
  public ShooterSubsystem(boolean isReal, PwmLEDs Leds) {
    setName("Shooter");

    if (isReal) {
      shooterIO = new ShooterIOReal(Leds);
    } else {
      shooterIO = new ShooterIOSim();
    }
  }

  //#region Control Methods

  /**
   * Runs the shooter motors
   * @param speed
   */
  public void runShooter(double speed) {
    shooterIO.RunShooter(speed);
  }

  public void runGreenWheel(double speed) {
    shooterIO.RunGreenWheel(speed);
  }

  /**
   * Stops the shooter motors
   */
  public void stopMotors() {
    shooterIO.StopMotors();
  }

  /**
   * Gets a boolean indicating whether a note is blocking the beam sensor
   * @return
   */
  public boolean isNoteLoaded() {
    return !shooterInputs.m_noteDetectorState;
  }

  public void setElevator(Value value) {
    shooterIO.SetElevator(value);
  }

  public void setElevatorUp() {
    setElevator(Value.kForward);
  }

  public void setElevatorDown() {
    setElevator(Value.kReverse);
  }

  //#endregion

  private boolean m_lastNoteDetectedValue = false;

  @Override
  public void periodic() {
    var newNoteDetectedValue = isNoteLoaded();
    if (newNoteDetectedValue != m_lastNoteDetectedValue) {
      if (newNoteDetectedValue && !m_lastNoteDetectedValue) {
        shooterIO.RunNoteDetectedLEDPattern();
      } else {
        shooterIO.ResetLEDs();
      }

      // Save the new value
      m_lastNoteDetectedValue = newNoteDetectedValue;
    }

    // Level2 Logging
    SmartDashboard.putNumber("Shooter/LaunchMotorOutput", shooterInputs.m_talonFXState);
    SmartDashboard.putNumber("Shooter/LaunchMotorVelocity", shooterInputs.m_talonFXVelocity);
    SmartDashboard.putNumber("Shooter/GuideMotorOutput", shooterInputs.m_talonFXVelocity);
    SmartDashboard.putBoolean("Shooter/NoteDetected", newNoteDetectedValue);
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
      shooterIO.RunShootingNoteLEDPattern();
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
      if (shooterInputs.m_elevationSolenoidState == Value.kForward) setElevatorDown(); else setElevatorUp();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Set_Elevation_Up",
      setElevationUpCommand(),
      "Set_Elevation_Down",
      setElevationDownCommand(),
      "Start_Shooting",
      startShootingNoteCommand(),
      "Stop_Shooting",
      stopMotorsCommand()
    );
  }
  //#endregion
}
