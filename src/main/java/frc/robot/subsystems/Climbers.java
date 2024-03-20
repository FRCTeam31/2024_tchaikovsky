package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.ClimbersConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase implements AutoCloseable {

  // Configuration
  private ClimbersConfig m_config;

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
    .withPosition(5, 4)
    .withSize(3, 2)
    .getEntry();

  // Motors
  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;

  // Limit Switches
  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;

  // Clutch Solenoids
  private DoubleSolenoid m_clutchSolenoidLeft;
  private DoubleSolenoid m_clutchSolenoidRight;

  // Member to track if the climb controls are enabled
  private boolean m_climbControlsEnabled = false;

  /**
   * Creates a new Climbers subsystem
   * @param config
   */
  public Climbers(ClimbersConfig config) {
    m_config = config;

    m_leftVictorSPX = new VictorSPX(config.VictorSPXLeftCanID);
    m_leftVictorSPX.configFactoryDefault();
    m_leftVictorSPX.setInverted(config.LeftInverted);
    m_leftVictorSPX.setNeutralMode(NeutralMode.Brake);

    m_rightVictorSPX = new VictorSPX(config.VictorSPXRightCanID);
    m_rightVictorSPX.configFactoryDefault();
    m_rightVictorSPX.setInverted(config.RightInverted);
    m_rightVictorSPX.setNeutralMode(NeutralMode.Brake);

    m_leftLimitSwitch = new DigitalInput(config.LeftLimitSwitchDIOChannel);
    m_rightLimitSwitch = new DigitalInput(config.RightLimitSwitchDIOChannel);

    m_clutchSolenoidLeft =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        m_config.LeftSolenoidForwardChannel,
        m_config.LeftSolenoidReverseChannel
      );
    m_clutchSolenoidRight =
      new DoubleSolenoid(
        30,
        PneumaticsModuleType.REVPH,
        m_config.RightSolenoidForwardChannel,
        m_config.RightSolenoidReverseChannel
      );
  }

  //#region Control Methods

  /**
   * Raises the desired climber arm
   * @param side The side to raise
   */
  public void raiseArm(Robot.Side side) {
    if (side == Robot.Side.kLeft && !m_leftLimitSwitch.get()) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, m_config.ClimberUpSpeed);
    }

    if (side == Robot.Side.kRight && !m_rightLimitSwitch.get()) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, m_config.ClimberUpSpeed);
    }
  }

  /**
   * Lowers the desired climber arm
   * @param side The side to lower
   * @param speed The speed to lower the arm at
   */
  public void lowerArm(Robot.Side side, double speed) {
    if (side == Robot.Side.kLeft) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
    }

    if (side == Robot.Side.kRight) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
    }
  }

  /**
   * Stops the desired climber arm
   * @param side
   */
  public void stopArm(Robot.Side side) {
    if (side == Robot.Side.kLeft) {
      m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
    }

    if (side == Robot.Side.kRight) {
      m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
    }
  }

  /**
   * Engages / disengages the desired clutch
   * @param side The side to engage / disengage
   * @param engaged Whether to engage or disengage the clutch
   */
  public void setClutch(Robot.Side side, boolean engaged) {
    if (side == Robot.Side.kLeft) {
      m_clutchSolenoidLeft.set(engaged ? Value.kForward : Value.kReverse);
    }

    if (side == Robot.Side.kRight) {
      m_clutchSolenoidRight.set(engaged ? Value.kForward : Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // d_leftLimitEntry.setBoolean(m_leftLimitSwitch.get());
    // d_rightLimitEntry.setBoolean(m_rightLimitSwitch.get());
    d_climbControlsActiveEntry.setBoolean(m_climbControlsEnabled);

    // Level2 Logging
    SmartDashboard.putBoolean("Climbers/ControlsEnabled", m_climbControlsEnabled);

    SmartDashboard.putNumber("Climbers/LeftMotorOutput", m_leftVictorSPX.getMotorOutputPercent());
    SmartDashboard.putNumber("Climbers/RightMotorOutput", m_rightVictorSPX.getMotorOutputPercent());

    SmartDashboard.putBoolean("Climbers/LeftLimitSwitch", m_leftLimitSwitch.get());
    SmartDashboard.putBoolean("Climbers/RightLimitSwitch", m_rightLimitSwitch.get());

    SmartDashboard.putString("Climbers/LeftClutchSolenoid", m_clutchSolenoidLeft.get().name());
    SmartDashboard.putString("Climbers/RightClutchSolenoid", m_clutchSolenoidRight.get().name());
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
            setClutch(Robot.Side.kRight, false);
            raiseArm(Robot.Side.kRight);
          } else {
            stopArm(Robot.Side.kRight);
          }

          // Raise left
          if (raiseLeftArm.getAsBoolean() && !m_leftLimitSwitch.get()) {
            setClutch(Robot.Side.kLeft, false);
            raiseArm(Robot.Side.kLeft);
          } else {
            stopArm(Robot.Side.kLeft);
          }

          // Lower Right
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Robot.Side.kRight, true);
            lowerArm(Robot.Side.kRight, MathUtil.applyDeadband(lowerRightArm.getAsDouble(), 0.1));
          }

          // Lower left
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Robot.Side.kLeft, true);
            lowerArm(Robot.Side.kLeft, MathUtil.applyDeadband(lowerLeftArm.getAsDouble(), 0.1));
          }
        }
      });
  }

  /**
   * Sequentially disengages the clutches and raises the arms until both limit switches have been hit.
   */
  public SequentialCommandGroup setArmsUpCommand() {
    return this.runOnce(() -> {
        setClutch(Robot.Side.kLeft, false);
        setClutch(Robot.Side.kRight, false);
      })
      .andThen(Commands.waitSeconds(0.075))
      .andThen(
        this.run(() -> {
            if (!m_leftLimitSwitch.get()) {
              setClutch(Robot.Side.kLeft, false);
              raiseArm(Robot.Side.kLeft);
            } else {
              setClutch(Robot.Side.kLeft, true);
              stopArm(Robot.Side.kLeft);
            }

            if (!m_rightLimitSwitch.get()) {
              setClutch(Robot.Side.kRight, false);
              raiseArm(Robot.Side.kRight);
            } else {
              setClutch(Robot.Side.kRight, true);
              stopArm(Robot.Side.kRight);
            }
          })
          .until(() -> m_leftLimitSwitch.get() && m_rightLimitSwitch.get())
          .finallyDo(() -> {
            stopArm(Robot.Side.kLeft);
            stopArm(Robot.Side.kRight);
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
