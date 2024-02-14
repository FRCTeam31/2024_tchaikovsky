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
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  private IntakeConfig m_config;
  private ShuffleboardTab d_intakeTab;
  private GenericEntry d_intakePositionEntry;
  private GenericEntry d_intakePidOutputEntry;

  private LazyCANSparkMax m_intakeRollerSparkMax;
  private LazyCANSparkMax m_intakeAngleSparkMaxLeft;
  private LazyCANSparkMax m_intakeAngleSparkMaxRight;
  private PIDController m_intakeAnglePid;

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public Intake(IntakeConfig config) {
    m_config = config;
    setName("Intake");
    d_intakeTab = Shuffleboard.getTab(getName());

    m_intakeRollerSparkMax =
      new LazyCANSparkMax(m_config.RollersCanId, MotorType.kBrushless);
    m_intakeRollerSparkMax.restoreFactoryDefaults();
    m_intakeRollerSparkMax.setInverted(m_config.RollersInverted);

    m_intakeAngleSparkMaxLeft =
      new LazyCANSparkMax(m_config.NeoLeftCanId, MotorType.kBrushless);
    m_intakeAngleSparkMaxLeft.restoreFactoryDefaults();
    m_intakeAngleSparkMaxLeft.setInverted(m_config.NeoLeftInverted);

    m_intakeAngleSparkMaxRight =
      new LazyCANSparkMax(m_config.NeoRightCanId, MotorType.kBrushless);
    m_intakeAngleSparkMaxRight.setInverted(m_config.NeoRightInverted);

    d_intakePositionEntry =
      d_intakeTab
        .add("Position (rotations)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Max", 15, "Min", -5))
        .getEntry();

    m_intakeAnglePid = new PIDController(0.1, 0, 0, 0.02);
    d_intakeTab
      .add("Steering PID", m_intakeAnglePid)
      .withWidget(BuiltInWidgets.kPIDController);
    d_intakePidOutputEntry =
      d_intakeTab
        .add("PID output", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Max", 2, "Min", -2))
        .getEntry();
  }

  //#region Methods
  @Override
  public void periodic() {
    d_intakePositionEntry.setDouble(getPosition());
  }

  // Gets the position of the Intake using the Encoder
  public double getPosition() {
    return m_intakeAngleSparkMaxRight.getEncoder().getPosition();
  }

  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    m_intakeRollerSparkMax.set(speed);
  }

  // Gives the motors used for chnaging the Intake Angle a speed
  public void setAngleMotorSpeed(double speed) {
    m_intakeAngleSparkMaxLeft.set(-speed);
    m_intakeAngleSparkMaxRight.set(speed);
  }

  // Method for setting a rotational setpoint for the intake motors to seek
  public void setIntakeRotation(double positionSetpoint) {
    var pidOutput = m_intakeAnglePid.calculate(getPosition(), positionSetpoint);

    d_intakePidOutputEntry.setDouble(pidOutput);

    var currentPosition = getPosition();
    if (currentPosition < m_config.AngleMaximum && pidOutput > 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 0.2));
    } else if (currentPosition > m_config.AngleMinimum && pidOutput < 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -0.2, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  public void close() {
    m_intakeAngleSparkMaxLeft.close();
    m_intakeAngleSparkMaxRight.close();
    m_intakeRollerSparkMax.close();
    m_intakeAnglePid.close();
  }

  //#endregion

  //#region Commands
  // Command for running the Intake to Intake a Note
  public Command runIntakeCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntakeRollers(speed.getAsDouble());
      });
  }

  // Command for changing the angle of the Position
  public Command setIntakeAngleCommand(double position) {
    return this.runOnce(() -> {
        setIntakeRotation(position);
      });
  }

  // Command for setting the Intake Angle
  public Command setIntakeAngleSpeed(DoubleSupplier speed) {
    return this.run(() -> {
        setAngleMotorSpeed(speed.getAsDouble());
      });
  }

  // Command for stopping the Intake Motors
  public Command stopAllMotorsCommand() {
    return this.run(() -> {
        m_intakeAngleSparkMaxLeft.stopMotor();
        m_intakeAngleSparkMaxRight.stopMotor();
        m_intakeRollerSparkMax.stopMotor();
      });
  }
  //#endregion
}
