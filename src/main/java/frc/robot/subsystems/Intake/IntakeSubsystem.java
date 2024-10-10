package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfig;
import frc.robot.subsystems.Intake.IIntakeIO.IntakeIOInputs;
import frc.robot.subsystems.Intake.IIntakeIO.IntakeIOOutputs;

import java.util.Map;
import java.util.function.DoubleSupplier;

import prime.control.PrimePIDConstants;
import prime.movers.LazyCANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  public class VMap {
    public static final int ROLLER_CAN_ID = 16;
    public static final int NEO_LEFT_CAN_ID = 15;
    public static final int NEO_RIGHT_CAN_ID = 14;

    public static final boolean ROLLERS_INVERTED = false;
    public static final boolean NEO_LEFT_INVERTED = false;
    public static final boolean NEO_RIGHT_INVERTED = true;

    public static final PrimePIDConstants INTAKE_ANGLE_PID = new PrimePIDConstants(0.05, 0, 0);
    public static final double POSITION_DELTA = 49;

    public static final int TOP_LIMIT_SWITCH_CHANNEL = 4;
    public static final int BOTTOM_LIMIT_SWITCH_CHANNEL = 5;
   }

  private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  private IIntakeIO intakeIO;
  private IntakeIOInputs intakeInputs = new IntakeIOInputs();
  private IntakeIOOutputs intakeOutputs = new IntakeIOOutputs();

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */

   

  public IntakeSubsystem(boolean isReal) {
    setName("Intake");
    if (isReal) {
      intakeIO = new IntakeIOReal();
    } else {
      intakeIO = new IntakeIOSim();
    }
    // Set the default command for the subsystem so that it runs the PID loop
    setDefaultCommand(seekAngleSetpointCommand());
  }

  //#region Control Methods

  /**
   * Gets the current position of the Intake Angle from the right NEO's encoder
   * @return Double
   */
  public double getPositionRight() {
    return intakeInputs.m_angleRightPosition;
    //return m_angleRight.getEncoder().getPosition();
  }

  /**
   * Gets the current position of the Intake Angle from the left NEO's encoder
   * @return Double
   */
  public double getPositionLeft() {
    return intakeInputs.m_angleLeftPosition;
    //return m_angleLeft.getEncoder().getPosition();
  }

  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    intakeIO.RunIntakeRollers(speed);
  }

  /**
   * Sets the speed of the Intake Angle Motors
   * @param speed
   */
  public void setAngleMotorSpeed(double speed) {
    intakeIO.SetAngleMotorSpeed(speed);
  }

  /**
   * Sets the Intake Angle to a given position in rotations of the motor shaft
   * @param positionSetpoint
   */
  public void setIntakeRotation() {
    var currentPosition = intakeInputs.m_angleRightPosition;
    var setpoint = intakeOutputs.m_angleToggledIn ? intakeOutputs.m_angleStartPoint : (intakeOutputs.m_angleStartPoint - VMap.POSITION_DELTA);
    SmartDashboard.putNumber("Intake/AngleSetpoint", setpoint);

    var pidOutput = intakeInputs.m_anglePidOutput;
    SmartDashboard.putNumber("Intake/AnglePIDOutput", pidOutput);

    // artificial limits
    if (currentPosition < intakeOutputs.m_angleStartPoint && pidOutput > 0 && !intakeInputs.m_topLimitSwitchState) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 1));
    } else if (
      currentPosition > (intakeOutputs.m_angleStartPoint - VMap.POSITION_DELTA) && pidOutput < 0 && !intakeInputs.m_bottomLimitSwitchState
    ) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -1, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  //#endregion

  @Override
  public void periodic() {
    intakeIO.setOutputs(intakeOutputs);
    intakeInputs = intakeIO.getInputs();

    // Level2 Logging
    SmartDashboard.putBoolean("Intake/ToggledIn", intakeOutputs.m_angleToggledIn);

    SmartDashboard.putNumber("Intake/ArmPositionRight", getPositionRight());
    SmartDashboard.putNumber("Intake/ArmPositionLeft", getPositionLeft());

    SmartDashboard.putNumber("Intake/RightMotorOutput", intakeInputs.m_angleRightState);
    SmartDashboard.putNumber("Intake/LeftMotorOutput", intakeInputs.m_angleLeftState);

    SmartDashboard.putNumber("Intake/RollersOutput", intakeInputs.m_rollersState);
  }

  //#region Commands

  /**
   * Constantly seeks the angle setpoint for the arm. If interrupted, stops the arm motors
   * @return none
   */
  public Command seekAngleSetpointCommand() {
    return this.run(() -> setIntakeRotation()).finallyDo(() -> stopArmMotorsCommand());
  }

  /**
   * Sets the rollers to a given speed
   */
  public Command setRollersSpeedCommand(DoubleSupplier speed) {
    return Commands.runOnce(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Runs the rollers at a given speed
   */
  public Command runRollersAtSpeedCommand(DoubleSupplier speed) {
    return Commands.run(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Sets the rollers to eject a note at max speed
   */
  public Command ejectNoteCommand() {
    return Commands.runOnce(() -> runIntakeRollers(-1));
  }

  /**
   * Sets the intake angle to loading position
   */
  public Command setIntakeInCommand() {
    return Commands.runOnce(() -> intakeOutputs.m_angleToggledIn = true);
  }

  /**
   * Sets the intake angle setpoint to ground position
   */
  public Command setIntakeOutCommand() {
    return Commands.runOnce(() -> intakeOutputs.m_angleToggledIn = false);
  }

  /**
   * Toggles the intake angle setpoint between in/out
   * @return
   */
  public Command toggleIntakeInAndOutCommand() {
    return Commands.runOnce(() -> intakeOutputs.m_angleToggledIn = !m_angleToggleDebouncer.calculate(intakeOutputs.m_angleToggledIn));
  }

  /**
   * Stops the intake arm motors
   * @return
   */
  public Command stopArmMotorsCommand() {
    return Commands.runOnce(() -> {
      intakeIO.StopMotors();
    });
  }

  /**
   * Stops the intake rollers
   * @return
   */
  public Command stopRollersCommand() {
    return Commands.runOnce(() -> {
      intakeIO.StopRollers();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Set_Intake_Out",
      setIntakeOutCommand(),
      "Set_Intake_In",
      setIntakeInCommand(),
      "Start_Note_Intake",
      setRollersSpeedCommand(() -> 1),
      "Stop_Note_Intake",
      stopRollersCommand(),
      "Eject_Note",
      ejectNoteCommand()
    );
  }
  //#endregion
}
