package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.config.IntakeConfig;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.movers.IPlannable;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase implements IPlannable {

  private IntakeConfig m_config;
  private DigitalInput m_topLimitSwitch;
  private DigitalInput m_bottomLimitSwitch;
  private LazyCANSparkMax m_rollers;
  private LazyCANSparkMax m_angleLeft;
  private LazyCANSparkMax m_angleRight;
  private PIDController m_anglePid;
  private double m_angleStartPoint;
  public boolean m_angleToggledIn;
  private Debouncer m_angleToggleDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // #region ShuffleBoard
  // private ShuffleboardTab d_intakeTab = Shuffleboard.getTab("Intake");

  // private GenericEntry d_positionLeftEntry = d_intakeTab
  //   .add("Position L (rotations)", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Max", 50, "Min", -50))
  //   .withPosition(6, 1)
  //   .withSize(2, 2)
  //   .getEntry();
  // private GenericEntry d_positionRightEntry = d_intakeTab
  //   .add("Position R (rotations)", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Max", 50, "Min", -50))
  //   .withPosition(8, 1)
  //   .withSize(2, 2)
  //   .getEntry();
  // private GenericEntry d_pidOutputEntry = d_intakeTab
  //   .add("PID output", 0)
  //   .withWidget(BuiltInWidgets.kNumberBar)
  //   .withProperties(Map.of("Max", 2, "Min", -2))
  //   .withPosition(4, 1)
  //   .withSize(2, 2)
  //   .getEntry();

  // private GenericEntry d_intakeSetpoint = d_intakeTab
  //   .add("Angle Toggled In", true)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .withPosition(2, 1)
  //   .withSize(1, 2)
  //   .getEntry();

  // private GenericEntry d_topLimitSwitch = d_intakeTab
  //   .add("Top LimitSwitch", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .getEntry();

  // private GenericEntry d_bottomLimitSwitch = d_intakeTab
  //   .add("Bottom Limitswitch", false)
  //   .withWidget(BuiltInWidgets.kBooleanBox)
  //   .getEntry();

  // #endregion

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public Intake(IntakeConfig config) {
    m_config = config;
    setName("Intake");
    m_topLimitSwitch = new DigitalInput(m_config.TopLimitSwitchChannel);
    m_bottomLimitSwitch = new DigitalInput(m_config.BottomLimitSwitchChannel);

    m_rollers = new LazyCANSparkMax(m_config.RollersCanId, MotorType.kBrushless);
    m_rollers.restoreFactoryDefaults();
    m_rollers.setInverted(m_config.RollersInverted);

    m_angleLeft = new LazyCANSparkMax(m_config.NeoLeftCanId, MotorType.kBrushless);
    m_angleLeft.restoreFactoryDefaults();
    m_angleLeft.setInverted(m_config.NeoLeftInverted);

    m_angleRight = new LazyCANSparkMax(m_config.NeoRightCanId, MotorType.kBrushless);
    m_angleRight.setInverted(m_config.NeoRightInverted);

    m_anglePid =
      new PIDController(m_config.IntakeAnglePid.kP, m_config.IntakeAnglePid.kI, m_config.IntakeAnglePid.kD, 0.02);
    m_angleStartPoint = getPositionRight();
    m_angleToggledIn = true;
    m_anglePid.setSetpoint(m_angleStartPoint);
    // d_intakeTab
    //   .add("Angle PID", m_anglePid)
    //   .withWidget(BuiltInWidgets.kPIDController)
    //   .withPosition(3, 1)
    //   .withSize(1, 2);
    // m_bottomLimitSwitch = new DigitalInput(config.BottomLimitSwitchChannel);
    // m_topLimitSwitch = new DigitalInput(config.TopLimitSwitchChannel);
  }

  //#region Control Methods

  /**
   * Gets the current position of the Intake Angle from the right NEO's encoder
   * @return
   */
  public double getPositionRight() {
    return m_angleRight.getEncoder().getPosition();
  }

  /**
   * Gets the current position of the Intake Angle from the left NEO's encoder
   * @return
   */
  public double getPositionLeft() {
    return m_angleLeft.getEncoder().getPosition();
  }

  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    m_rollers.set(speed);
  }

  /**
   * Sets the speed of the Intake Angle Motors
   * @param speed
   */
  public void setAngleMotorSpeed(double speed) {
    m_angleLeft.set(-speed);
    m_angleRight.set(speed);
  }

  /**
   * Sets the Intake Angle to a given position in rotations of the motor shaft
   * @param positionSetpoint
   */
  public void setIntakeRotation() {
    var currentPosition = getPositionRight();

    var setpoint = m_angleToggledIn ? m_angleStartPoint : (m_angleStartPoint - m_config.PositionDelta);

    var pidOutput = m_anglePid.calculate(currentPosition, setpoint);

    // d_pidOutputEntry.setDouble(pidOutput);
    // artificial limits
    if (currentPosition < m_angleStartPoint && pidOutput > 0 && !m_topLimitSwitch.get()) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 1));
    } else if (
      currentPosition > (m_angleStartPoint - m_config.PositionDelta) && pidOutput < 0 && !m_bottomLimitSwitch.get()
    ) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -1, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  //#endregion

  @Override
  public void periodic() {
    // d_positionLeftEntry.setDouble(getPositionLeft());
    // d_positionRightEntry.setDouble(getPositionRight());
    // d_intakeSetpoint.setBoolean(m_angleToggledIn);
    // d_bottomLimitSwitch.setBoolean(m_bottomLimitSwitch.get());
    // d_topLimitSwitch.setBoolean(m_topLimitSwitch.get());
  }

  //#region Commands

  /**
   * Command for running the Intake to Intake a Note
   */
  public Command setRollersSpeedCommand(DoubleSupplier speed) {
    return Commands.runOnce(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Command for running the Intake to Intake a Note at a variable speed
   */
  public Command runRollersWithSpeedCommand(DoubleSupplier speed) {
    return Commands.run(() -> runIntakeRollers(speed.getAsDouble()));
  }

  /**
   * Command for running the Intake to Eject a Note
   */
  public Command ejectNoteCommand() {
    return Commands.runOnce(() -> runIntakeRollers(-1));
  }

  // Seeks an Angle Setpoint
  public Command seekAngleSetpointCommand() {
    return this.run(() -> setIntakeRotation());
  }

  /**
   * Command for setting the intake angle into loading position
   */
  public Command setIntakeInCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = true);
  }

  /**
   * Command for setting the intake angle into ground position
   */
  public Command setIntakeOutCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = false);
  }

  /**
   * Toggles the intake angle setpoint between in/out
   * @return
   */
  public Command toggleIntakeInAndOutCommand() {
    return Commands.runOnce(() -> m_angleToggledIn = !m_angleToggleDebouncer.calculate(m_angleToggledIn));
  }

  // Command for stopping the Intake Motors
  public Command stopArmMotorsCommand() {
    return Commands.runOnce(() -> {
      m_angleLeft.stopMotor();
      m_angleRight.stopMotor();
    });
  }

  // Command for stopping the Intake Motors
  public Command stopRollersCommand() {
    return Commands.runOnce(() -> {
      m_rollers.stopMotor();
    });
  }

  public Command intakeNoteForTime(double seconds, double speed) {
    return setRollersSpeedCommand(() -> speed).andThen(new WaitCommand(seconds)).andThen(stopRollersCommand());
  }

  public Command outtakeNoteForTime(double seconds, double speed) {
    return Commands
      .runOnce(() -> {
        setRollersSpeedCommand(() -> speed);
      })
      .andThen(new WaitCommand(seconds))
      .andThen(stopRollersCommand());
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
      stopRollersCommand()
    );
  }

  //#endregion

  /**
   * Closes the Intake
   */
  public void close() {
    m_angleLeft.close();
    m_angleRight.close();
    m_rollers.close();
    m_anglePid.close();
  }
}
