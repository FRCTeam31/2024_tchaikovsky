package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Climbers extends SubsystemBase {

  private VictorSPX m_leftVictorSPX;
  private VictorSPX m_rightVictorSPX;
  private boolean toggleClimber = false;

  // Creates the Climbers
  public Climbers(RobotConfig robotConfig) {
    m_leftVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXLeftCanID);
    m_rightVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXRightCanID);
  }

  //#region Methods
  // Raises the Left Climber
  public void raiseLeftArm() {
    m_leftVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      RobotConfig.m_climbSpeed
    );
  }

  // Raises the Right Climber
  public void raiseRightArm() {
    m_rightVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      RobotConfig.m_climbSpeed
    );
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

  //#region Commands
  // Command for toggling the climbers on and off
  public Command toggleClimbersCommand() {
    return this.runOnce(() -> {
        if (!toggleClimber) {
          toggleClimber = true;
        } else {
          toggleClimber = false;
        }
      });
  }

  // Command for raising the Left Arm
  public Command raiseLeftArmCommand() {
    return this.run(() -> {
        if (toggleClimber) {
          raiseLeftArm();
        }
      });
  }

  // Command  for raising the Right Arm
  public Command raiseRightArmCommand() {
    return this.run(() -> {
        if (toggleClimber) {
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
        if (toggleClimber) {
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
