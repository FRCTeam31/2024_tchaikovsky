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
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase {

  private ShuffleboardTab d_tab = Shuffleboard.getTab("Climbers");
  private GenericEntry d_leftClimberEntry = d_tab
    .add("Left Climber Limit Switch", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_rightClimberEntry = d_tab
    .add("Right Climber Limit Switch", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  private GenericEntry d_climbControlsActiveEntry = d_tab
    .add("Climb Controls Active", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;
  private DigitalInput m_leftLimitSwitch;
  private DigitalInput m_rightLimitSwitch;

  private boolean m_toggleClimber = false;

  // Creates the Climbers
  public Climbers(RobotConfig robotConfig) {
    m_leftVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXLeftCanID);
    m_rightVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXRightCanID);
    m_leftLimitSwitch = new DigitalInput(0);
    m_leftLimitSwitch = new DigitalInput(1);
  }

  //#region Methods
  // Raises the Left Climber
  public void raiseLeftArm() {
    if (!m_leftLimitSwitch.get()) {
      m_leftVictorSPX.set(
        VictorSPXControlMode.PercentOutput,
        RobotConfig.m_climbSpeed
      );
    } else {
      stopLeftArm();
    }
  }

  // Raises the Right Climber
  public void raiseRightArm() {
    if (!m_rightLimitSwitch.get()) {
      m_rightVictorSPX.set(
        VictorSPXControlMode.PercentOutput,
        RobotConfig.m_climbSpeed
      );
    } else {
      stopRightArm();
    }
  }

  // Lowers the Left Climber
  public void lowerLeftArm() {
    m_leftVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -RobotConfig.m_climbSpeed
    );
  }

  // Lowers the Right Climber
  public void lowerRightArm() {
    m_rightVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -RobotConfig.m_climbSpeed
    );
  }

  // Stops the Right CLimber Motor
  public void stopLeftArm() {
    m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  // Stops the Left Climber Motor
  public void stopRightArm() {
    m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  //#endregion

  @Override
  public void periodic() {
    d_leftClimberEntry.setBoolean(m_leftLimitSwitch.get());
    d_rightClimberEntry.setBoolean(m_rightLimitSwitch.get());
    d_climbControlsActiveEntry.setBoolean(m_toggleClimber);
  }

  //#region Commands
  // Command for toggling the climbers on and off
  public Command toggleClimbersCommand() {
    return this.runOnce(() -> {
        if (!m_toggleClimber) {
          m_toggleClimber = true;
        } else {
          m_toggleClimber = false;
        }
      });
  }

  // Command for raising the Left Arm
  public Command raiseLeftArmCommand() {
    return this.run(() -> {
        if (m_toggleClimber) {
          raiseLeftArm();
        }
      });
  }

  // Command  for raising the Right Arm
  public Command raiseRightArmCommand() {
    return this.run(() -> {
        if (m_toggleClimber) {
          raiseRightArm();
        }
      });
  }

  // Command for lowering the Climbers
  public Command LowerClimbersCommand(
    DoubleSupplier leftClimber,
    DoubleSupplier rightClimber
  ) {
    return this.run(() -> {
        if (m_toggleClimber) {
          if (leftClimber.getAsDouble() > 0.5) {
            lowerLeftArm();
          } else if (rightClimber.getAsDouble() > 0.5) {
            lowerRightArm();
          }
        }
      });
  }

  // Command for stopping the Left Arm motor
  public Command stopLeftArmCommand() {
    return this.run(() -> {
        stopLeftArm();
      });
  }

  // Command for stopping the right Arm motor
  public Command stopRightArmCommand() {
    return this.run(() -> {
        stopRightArm();
      });
  }
  //#endregion
}
