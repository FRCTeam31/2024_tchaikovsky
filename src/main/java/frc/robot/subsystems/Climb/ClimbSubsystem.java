package frc.robot.subsystems.Climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriverDashboard;
import frc.robot.subsystems.Climb.IClimbIO.Side;
import frc.robot.subsystems.Climb.IClimbIO.ClimbIOInputs;
import frc.robot.subsystems.Climb.IClimbIO.ClimbIOOutputs;
import frc.robot.subsystems.Climb.IClimbIO.Direction;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimbSubsystem extends SubsystemBase {
  public class VMap {
    public static final int VICTORSPX_LEFT_CAN_ID = 18;
    public static final int VICTORSPX_RIGHT_CAN_ID = 17;

    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = true;

    public static final double CLIMBER_UP_SPEED = 0.5;
    public static final double CLIMBER_DOWN_SPEED = -1;

    public static final int LEFT_LIMIT_SWITCH_DIO_CHANNEL = 2;
    public static final int RIGHT_LIMIT_SWITCH_DIO_CHANNEL = 3;

    public static final int LEFT_SOLENOID_FORWARD_CHANNEL = 2;
    public static final int LEFT_SOLENOID_REVERSE_CHANNEL = 9;
    public static final int RIGHT_SOLENOID_FORWARD_CHANNEL = 10;
    public static final int RIGHT_SOLENOID_REVERSE_CHANNEL = 11;
  }

  private IClimbIO climbIO;
  private ClimbIOInputs climbInputs = new ClimbIOInputs();
  private ClimbIOOutputs climbOutputs = new ClimbIOOutputs();

  private DriverDashboard m_driverDashboard;
  
  // Member to track if the climb controls are enabled
  private boolean m_climbControlsEnabled = false;

  /**
   * Creates a new Climbers subsystem
   * @param dashboard
   */
  public ClimbSubsystem(boolean isReal, DriverDashboard dashboard) {
    setName("Climb");

    if (isReal) {
      climbIO = new ClimbIOReal();
    } else {
      climbIO = new ClimbIOSim();
    }
  }

  //#region Control Methods
  /**
   * Controls the movement of the desired climb arm
   * @param side The side to move
   * @param direction The direction of travel
   * @param speed The speed at which to travel (Only set up for downward direction)
   */
  public void moveArm(Side side, Direction direction, double speed) {
    climbIO.MoveArm(side, direction, speed);
  }

  /**
   * Stops the desired climber arm and engages the clutch
   * @param side
   */
  public void stopArm(Side side) {
      climbIO.StopArm(side);
      climbIO.SetClutch(side, true);
  }

  /**
   * Engages / disengages the desired clutch
   * @param side The side to engage / disengage
   * @param engaged Whether to engage or disengage the clutch
   */
  public void setClutch(Side side, boolean engaged) {
    climbIO.SetClutch(side, engaged);
  }

  @Override
  public void periodic() {
    // d_leftLimitEntry.setBoolean(m_leftLimitSwitch.get());
    // d_rightLimitEntry.setBoolean(m_rightLimitSwitch.get());
    m_driverDashboard.ClimberControlsActiveBox.setBoolean(m_climbControlsEnabled);

    // Level2 Logging
    SmartDashboard.putBoolean("Climbers/ControlsEnabled", m_climbControlsEnabled);

    SmartDashboard.putNumber("Climbers/LeftMotorOutput", climbInputs.m_leftVictorSPXOutputPercent);
    SmartDashboard.putNumber("Climbers/RightMotorOutput", climbInputs.m_rightVictorSPXOutputPercent);

    SmartDashboard.putBoolean("Climbers/LeftLimitSwitch", climbInputs.m_leftLimitSwitchState);
    SmartDashboard.putBoolean("Climbers/RightLimitSwitch", climbInputs.m_rightLimitSwitchState);
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
          var raiseRightArmTriggered = raiseRightArm.getAsBoolean();
          var rightLimitSwitchTriggered = climbInputs.m_rightLimitSwitchState;
          if (raiseRightArmTriggered && !rightLimitSwitchTriggered) {
            setClutch(Side.kRight, false);
            moveArm(Side.kRight, Direction.kRaise, 0);
          } else {
            stopArm(Side.kRight);
          }

          // Raise left
          var raiseLeftArmTriggered = raiseLeftArm.getAsBoolean();
          var leftLimitSwitchTriggered = climbInputs.m_leftLimitSwitchState;
          if (raiseLeftArmTriggered && !leftLimitSwitchTriggered) {
            setClutch(Side.kLeft, false);
            moveArm(Side.kLeft, Direction.kRaise, 0);
          } else {
            stopArm(Side.kLeft);
          }

          // Lower Right
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Side.kRight, true);
            moveArm(Side.kRight, Direction.kLower, MathUtil.applyDeadband(lowerRightArm.getAsDouble(), 0.1));
          }

          // Lower left
          if (!raiseRightArm.getAsBoolean() && !raiseLeftArm.getAsBoolean()) {
            setClutch(Side.kLeft, true);
            moveArm(Side.kLeft, Direction.kLower, MathUtil.applyDeadband(lowerLeftArm.getAsDouble(), 0.1));
          }
        }
      });
  }

  /**
   * Sequentially disengages the clutches and raises the arms until both limit switches have been hit.
   */
  public SequentialCommandGroup setArmsUpCommand() {
    return this.runOnce(() -> {
        setClutch(Side.kLeft, false);
        setClutch(Side.kRight, false);
      })
      .andThen(Commands.waitSeconds(0.075))
      .andThen(
        this.runOnce(() -> {
              moveArm(Side.kLeft, Direction.kRaise, 0);
              moveArm(Side.kRight, Direction.kRaise, 0);
          })
      )
      .andThen(new WaitUntilCommand(() -> climbInputs.m_leftLimitSwitchState || climbInputs.m_rightLimitSwitchState).withTimeout(2))
      .andThen(() -> {
        stopArm(Side.kLeft);
        stopArm(Side.kRight);
      });
  }
  //#endregion
}
