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

  public Climbers(RobotConfig robotConfig) {
    m_leftVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXLeftCanID);
    m_rightVictorSPX = new VictorSPX(robotConfig.m_climbersVictorSPXRightCanID);
  }

  //#region Methods
  public void raiseLeftArm() {
    m_leftVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      RobotConfig.m_climbSpeed
    );
  }

  public void raiseRightArm() {
    m_rightVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      RobotConfig.m_climbSpeed
    );
  }

  public void lowerLeftArm() {
    m_leftVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -RobotConfig.m_climbSpeed
    );
  }

  public void lowerRightArm() {
    m_rightVictorSPX.set(
      VictorSPXControlMode.PercentOutput,
      -RobotConfig.m_climbSpeed
    );
  }

  public void stopLeftArm() {
    m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stopRightArm() {
    m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  //#endregion

  //#region Commands

  public Command toggleClimbersCommand() {
    return this.runOnce(() -> {
        if (!toggleClimber) {
          toggleClimber = true;
        } else {
          toggleClimber = false;
        }
      });
  }

  public Command raiseLeftArmCommand() {
    return this.run(() -> {
        if (toggleClimber) {
          raiseLeftArm();
        }
      });
  }

  public Command raiseRightArmCommand() {
    return this.run(() -> {
        if (toggleClimber) {
          raiseRightArm();
        }
      });
  }

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

  public Command stopLeftArmCommand() {
    return this.run(() -> {
        stopLeftArm();
      });
  }

  public Command stopRightArmCommand() {
    return this.run(() -> {
        stopRightArm();
      });
  }
  //#endregion
}
