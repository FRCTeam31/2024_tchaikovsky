package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ClimbersConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase implements AutoCloseable {

  private ClimbersConfig m_config;

  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;
  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;
  private DoubleSolenoid m_clutchSolenoidLeft;
  private DoubleSolenoid m_clutchSolenoidRight;

  private boolean m_climbControlsEnabled = false;

  private ShuffleboardTab d_tab = Shuffleboard.getTab("Driver");
  // private GenericEntry d_leftLimitEntry = d_tab
  //   .add("Left Climber Limit Switch", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .withPosition(1, 1)
  //   .withSize(2, 1)
  //   .getEntry();
  // private GenericEntry d_rightLimitEntry = d_tab
  //   .add("Right Climber Limit Switch", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .withPosition(1, 2)
  //   .withSize(2, 1)
  //   .getEntry();
  public GenericEntry d_climbControlsActiveEntry = d_tab
    .add("Climbers Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(4, 4)
    .withSize(3, 1)
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

    m_clutchSolenoidLeft =
      new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        m_config.LeftSolenoidForwardChannel,
        m_config.LeftSolenoidReverseChannel
      );
    m_clutchSolenoidRight =
      new DoubleSolenoid(
        PneumaticsModuleType.REVPH,
        m_config.RightSolenoidForwardChannel,
        m_config.RightSolenoidReverseChannel
      );
  }

  //#region Control Methods

  /**
   * Raises the Left Climber
   */
  public void raiseLeftArm() {
    m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, m_config.ClimberUpSpeed);
  }

  /**
   * Raises the Right Climber
   */
  public void raiseRightArm() {
    m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, m_config.ClimberUpSpeed);
  }

  /**
   * Lowers the Left Climber
   */
  public void lowerLeftArm(double speed) {
    m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
  }

  /**
   * Lowers the Right Climber
   */
  public void lowerRightArm(double speed) {
    m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
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

  /**
   * Engages / disengages the left clutch
   * @param engaged
   */
  public void setLeftClutch(boolean engaged) {
    m_clutchSolenoidLeft.set(engaged ? Value.kForward : Value.kReverse);
  }

  /**
   * Engages / disengages the right clutch
   * @param engaged
   */
  public void setRightClutch(boolean engaged) {
    m_clutchSolenoidRight.set(engaged ? Value.kForward : Value.kReverse);
  }

  @Override
  public void periodic() {
    // d_leftLimitEntry.setBoolean(m_leftLimitSwitch.get());
    // d_rightLimitEntry.setBoolean(m_rightLimitSwitch.get());
    d_climbControlsActiveEntry.setBoolean(m_climbControlsEnabled);
  }

  //#endregion

  //#region Commands

  /**
   * Toggles the Climbers Controls
   */
  public Command toggleClimbControlsCommand() {
    return Commands.runOnce(() -> {
      m_climbControlsEnabled = !m_climbControlsEnabled;
    });
  }

  /**
   * Continually raises / lowers the two arms based on controller inputs
   */
  public Command defaultClimbingCommand(
    BooleanSupplier raiseRightArm,
    BooleanSupplier raiseLeftArm,
    DoubleSupplier lowerRightArm,
    DoubleSupplier lowerLeftArm
  ) {
    return this.run(() -> {
        // Raise only if climbing controls is enabled
        if (m_climbControlsEnabled) {
          // Raise Right
          if (raiseRightArm.getAsBoolean() && !m_rightLimitSwitch.get()) {
            setRightClutch(false);
            raiseRightArm();
          } else {
            stopRightArm();
          }

          // Raise left
          if (raiseLeftArm.getAsBoolean() && !m_leftLimitSwitch.get()) {
            setLeftClutch(false);
            raiseLeftArm();
          } else {
            stopLeftArm();
          }

          // Lower Right
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setRightClutch(true);
            lowerRightArm(MathUtil.applyDeadband(lowerRightArm.getAsDouble(), 0.1));
          }

          // Lower left
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setLeftClutch(true);
            lowerLeftArm(MathUtil.applyDeadband(lowerLeftArm.getAsDouble(), 0, 1));
          }
        }
      });
  }

  /**
   * Sequentially disengages the clutches and raises the arms until both limit switches have been hit.
   */
  public SequentialCommandGroup setArmsUpCommand() {
    return this.runOnce(() -> {
        setLeftClutch(false);
        setRightClutch(false);
      })
      .andThen(Commands.waitSeconds(0.075))
      .andThen(
        this.run(() -> {
            if (!m_leftLimitSwitch.get()) {
              raiseLeftArm();
            } else {
              stopLeftArm();
            }

            if (!m_rightLimitSwitch.get()) {
              raiseRightArm();
            } else {
              stopRightArm();
            }
          })
          .until(() -> m_leftLimitSwitch.get() && m_rightLimitSwitch.get())
          .finallyDo(() -> {
            stopLeftArm();
            stopRightArm();
          })
      );
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
