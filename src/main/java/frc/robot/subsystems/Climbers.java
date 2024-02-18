package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ClimbersConfig;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase implements AutoCloseable {

  private ClimbersConfig m_config;

  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;
  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;

  private boolean m_climbControlsEnabled = false;

  private ShuffleboardTab d_tab = Shuffleboard.getTab("Climbers");
  private GenericEntry d_leftLimitEntry = d_tab
    .add("Left Climber Limit Switch", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_rightLimitEntry = d_tab
    .add("Right Climber Limit Switch", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_climbControlsActiveEntry = d_tab
    .add("Climb Controls Active", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  /**
   * Creates a new Climbers subsystem
   * @param config
   */
  public Climbers(ClimbersConfig config) {
    m_config = config;

    m_leftVictorSPX = new VictorSPX(config.VictorSPXLeftCanID);
    m_leftVictorSPX.configFactoryDefault();
    m_leftVictorSPX.setInverted(config.LeftInverted);

    m_rightVictorSPX = new VictorSPX(config.VictorSPXRightCanID);
    m_rightVictorSPX.configFactoryDefault();
    m_rightVictorSPX.setInverted(config.RightInverted);

    m_leftLimitSwitch = new DigitalInput(config.LeftLimitSwitchDIOChannel);
    m_rightLimitSwitch = new DigitalInput(config.RightLimitSwitchDIOChannel);
  }

  //#region Control Methods

  /**
   * Raises the Left Climber
   */
  public void raiseLeftArm() {
    if (!m_leftLimitSwitch.get()) {
      m_leftVictorSPX.set(
        VictorSPXControlMode.PercentOutput,
        m_config.ClimbersSpeed
      );
    } else {
      stopLeftArm();
    }
  }

  /**
   * Raises the Right Climber
   */
  public void raiseRightArm() {
    if (!m_rightLimitSwitch.get()) {
      m_rightVictorSPX.set(
        VictorSPXControlMode.PercentOutput,
        m_config.ClimbersSpeed
      );
    } else {
      stopRightArm();
    }
  }

  /**
   * Lowers the Left Climber
   */
  public void lowerLeftArm() {
    m_leftVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -m_config.ClimbersSpeed
    );
  }

  /**
   * Lowers the Right Climber
   */
  public void lowerRightArm() {
    m_rightVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -m_config.ClimbersSpeed
    );
  }

  /**
   * Stops the Left Climber Motor
   */
  public void stopLeftArm() {
    m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  /**
   * Stops the Right Climber Motor
   */
  public void stopRightArm() {
    m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  //#endregion

  @Override
  public void periodic() {
    d_leftLimitEntry.setBoolean(m_leftLimitSwitch.get());
    d_rightLimitEntry.setBoolean(m_rightLimitSwitch.get());
    d_climbControlsActiveEntry.setBoolean(m_climbControlsEnabled);
  }

  //#region Commands

  /**
   * Toggles the Climbers Controls
   * @return
   */
  public Command toggleClimbersCommand() {
    return this.runOnce(() -> {
        m_climbControlsEnabled = !m_climbControlsEnabled;
      });
  }

  /**
   * Command for raising the Left Arm
   * @return
   */
  public Command raiseLeftArmCommand() {
    return this.run(() -> {
        if (m_climbControlsEnabled) {
          raiseLeftArm();
        }
      });
  }

  /**
   * Command for raising the Right Arm
   * @return
   */
  public Command raiseRightArmCommand() {
    return this.run(() -> {
        if (m_climbControlsEnabled) {
          raiseRightArm();
        }
      });
  }

  /**
   * Command for lowering the left climber
   * @return
   */
  public Command lowerLeftArmCommand() {
    return this.run(() -> {
        if (m_climbControlsEnabled) {
          lowerLeftArm();
        }
      });
  }

  /**
   * Command for lowering the right climber
   * @return
   */
  public Command lowerRightArmCommand() {
    return this.run(() -> {
        if (m_climbControlsEnabled) {
          lowerRightArm();
        }
      });
  }

  /**
   * Command for lowering both climbers
   * @return
   */
  public Command LowerClimbersCommand(
    DoubleSupplier leftClimber,
    DoubleSupplier rightClimber
  ) {
    return this.run(() -> {
        if (m_climbControlsEnabled) {
          if (leftClimber.getAsDouble() > 0.5) {
            lowerLeftArm();
          } else if (rightClimber.getAsDouble() > 0.5) {
            lowerRightArm();
          }
        }
      });
  }

  /**
   * Command for stopping the left Arm motor
   * @return
   */
  public Command stopLeftArmCommand() {
    return this.run(() -> {
        stopLeftArm();
      });
  }

  /**
   * Command for stopping the right Arm motor
   * @return
   */
  public Command stopRightArmCommand() {
    return this.run(() -> {
        stopRightArm();
      });
  }

  //#endregion

  @Override
  public void close() {
    m_leftVictorSPX.DestroyObject();
    m_rightVictorSPX.DestroyObject();
    m_leftLimitSwitch.close();
    m_rightLimitSwitch.close();
  }
}
