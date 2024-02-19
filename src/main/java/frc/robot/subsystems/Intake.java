package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfig;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  private IntakeConfig m_config;

  private LazyCANSparkMax m_rollers;
  private LazyCANSparkMax m_angleLeft;
  private LazyCANSparkMax m_angleRight;
  private PIDController m_anglePid;
  private double m_angleSetpoint;

  private ShuffleboardTab d_intakeTab = Shuffleboard.getTab("Intake");

  private GenericEntry d_positionLeftEntry = d_intakeTab
    .add("Position L (rotations)", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Max", 50, "Min", -50))
    .getEntry();
  private GenericEntry d_positionRightEntry = d_intakeTab
    .add("Position R (rotations)", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Max", 50, "Min", -50))
    .getEntry();
  private GenericEntry d_pidOutputEntry = d_intakeTab
    .add("PID output", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Max", 2, "Min", -2))
    .getEntry();

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public Intake(IntakeConfig config) {
    m_config = config;
    setName("Intake");

    m_rollers =
      new LazyCANSparkMax(m_config.RollersCanId, MotorType.kBrushless);
    m_rollers.restoreFactoryDefaults();
    m_rollers.setInverted(m_config.RollersInverted);

    m_angleLeft =
      new LazyCANSparkMax(m_config.NeoLeftCanId, MotorType.kBrushless);
    m_angleLeft.restoreFactoryDefaults();
    m_angleLeft.setInverted(m_config.NeoLeftInverted);

    m_angleRight =
      new LazyCANSparkMax(m_config.NeoRightCanId, MotorType.kBrushless);
    m_angleRight.setInverted(m_config.NeoRightInverted);

    m_anglePid =
      new PIDController(
        m_config.IntakeAnglePid.kP,
        m_config.IntakeAnglePid.kI,
        m_config.IntakeAnglePid.kD,
        0.02
      );
    m_angleSetpoint = m_config.PositionMaximum;
    m_anglePid.setSetpoint(m_angleSetpoint);
    d_intakeTab
      .add("Steering PID", m_anglePid)
      .withWidget(BuiltInWidgets.kPIDController);
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
  public void setIntakeRotation(double positionSetpoint) {
    var currentPosition = getPositionRight();
    var pidOutput = m_anglePid.calculate(currentPosition, positionSetpoint);
    // pidOutput = Controls.linearScaledDeadband(pidOutput, 0.01);

    d_pidOutputEntry.setDouble(pidOutput);

    if (currentPosition < m_config.PositionMaximum && pidOutput > 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 0.5));
    } else if (currentPosition > m_config.PositionMinimum && pidOutput < 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -0.5, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  //#endregion

  @Override
  public void periodic() {
    d_positionLeftEntry.setDouble(getPositionLeft());
    d_positionRightEntry.setDouble(getPositionRight());
  }

  //#region Commands

  /**
   *  Command for running the Intake to Intake a Note
   */
  public Command intakeNoteCommand() {
    return this.run(() -> runIntakeRollers(0.5));
  }

  /**
   * Command for running the Intake to Eject a Note
   */
  public Command ejectNoteCommand() {
    return this.run(() -> runIntakeRollers(-0.5));
  }

  public Command seekAngleSetpointCommand() {
    return this.run(() -> setIntakeRotation(m_angleSetpoint));
  }

  /**
   * Command for setting the intake angle into pickup position
   */
  public Command setIntakeOutCommand() {
    return this.runOnce(() -> m_angleSetpoint = m_config.PositionMinimum);
  }

  /**
   * Command for setting the intake angle into loading position
   */
  public Command setIntakeInCommand() {
    return this.runOnce(() -> m_angleSetpoint = m_config.PositionMaximum);
  }

  // Command for stopping the Intake Motors
  public Command stopAllMotorsCommand() {
    return this.run(() -> {
        m_angleLeft.stopMotor();
        m_angleRight.stopMotor();
        m_rollers.stopMotor();
      });
  }

  public Command defaultIntakeCommand(
    DoubleSupplier intakeNote,
    BooleanSupplier ejectNote,
    BooleanSupplier intakeIn,
    BooleanSupplier intakeOut
  ) {
    return this.run(() -> {
        seekAngleSetpointCommand();
        if (!ejectNote.getAsBoolean()) {
          runIntakeRollers(intakeNote.getAsDouble());
        } else if (ejectNote.getAsBoolean()) {
          runIntakeRollers(1);
        }

        if (intakeIn.getAsBoolean()) {
          setIntakeInCommand();
        } else if (intakeOut.getAsBoolean()) {
          setIntakeOutCommand();
        }
      });
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
