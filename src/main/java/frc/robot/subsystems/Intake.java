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
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  private RobotConfig m_robotConfig;
  private ShuffleboardTab d_intakeTab;
  private GenericEntry d_intakePositionEntry;
  private GenericEntry d_intakePidOutputEntry;

  private LazyCANSparkMax m_intakeRollerSparkMax;
  private LazyCANSparkMax m_intakeAngleSparkMaxLeft;
  private LazyCANSparkMax m_intakeAngleSparkMaxRight;
  private PIDController m_intakeAnglePid;

  // TODO: move these into config

  // Creates a new Intake
  public Intake(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    setName("Intake");
    d_intakeTab = Shuffleboard.getTab(getName());

    m_intakeRollerSparkMax =
      new LazyCANSparkMax(
        m_robotConfig.m_intakeRollerSparkMaxCanID,
        MotorType.kBrushless
      );

    m_intakeAngleSparkMaxLeft =
      new LazyCANSparkMax(
        m_robotConfig.m_intakeAngleSparkMaxLeftCanID,
        MotorType.kBrushless
      );
    m_intakeAngleSparkMaxRight =
      new LazyCANSparkMax(
        m_robotConfig.m_intakeAngleSparkMaxRightCanID,
        MotorType.kBrushless
      );
    m_intakeAngleSparkMaxRight.setInverted(true);

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

  @Override
  public void periodic() {
    d_intakePositionEntry.setDouble(getPosition());
  }

  public double getPosition() {
    return m_intakeAngleSparkMaxRight.getEncoder().getPosition();
  }

  // Method for giving the Intake Roller Motor a speed
  public void runIntakeRollers(double speed) {
    m_intakeRollerSparkMax.set(speed);
  }

  public void setAngleMotorSpeed(double speed) {
    m_intakeAngleSparkMaxLeft.set(speed);
    m_intakeAngleSparkMaxLeft.set(speed);
  }

  // public void setIntakeAngle(Rotation2d rotation2d) {
  //   var desiredRotation = rotation2d.getRotations();
  //   var desiredRotationWithRatio = desiredRotation * 20;

  //   setIntakeRotation();
  // }

  // Method for setting a rotational setpoint for the intake motors to seek
  public void setIntakeRotation(RobotConfig robotConfig) {
    var pidOutput = m_intakeAnglePid.calculate(
      getPosition(),
      robotConfig.m_positionSetpoint
    );

    d_intakePidOutputEntry.setDouble(pidOutput);

    var currentPosition = getPosition();
    if (currentPosition < robotConfig.m_upperLimit && pidOutput > 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, 0, 0.2));
    } else if (currentPosition > robotConfig.m_lowerLimit && pidOutput < 0) {
      setAngleMotorSpeed(MathUtil.clamp(pidOutput, -0.2, 0));
    } else {
      setAngleMotorSpeed(0);
    }
  }

  //#region Commands
  // Command for running the intake to intake a Note
  public Command runIntakeCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntakeRollers(speed.getAsDouble());
      });
  }

  // Command for changing the angle of the Position
  public Command setIntakeAngleCommand(
    double position,
    RobotConfig robotConfig
  ) {
    return this.runOnce(() -> {
        robotConfig.m_positionSetpoint = position;
      });
  }

  public Command runIntakeAnglePid() {
    return this.run(() -> setIntakeRotation(m_robotConfig));
  }

  public Command setIntakeAngleSpeed(DoubleSupplier speed) {
    return this.run(() -> {
        setAngleMotorSpeed(speed.getAsDouble());
      });
  }

  // Command for stopping the Intake Motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        m_intakeAngleSparkMaxLeft.stopMotor();
        m_intakeAngleSparkMaxRight.stopMotor();
        m_intakeRollerSparkMax.stopMotor();
      });
  }

  //#endregion

  public void close() {
    m_intakeAngleSparkMaxLeft.close();
    m_intakeAngleSparkMaxRight.close();
    m_intakeRollerSparkMax.close();
    m_intakeAnglePid.close();
  }
}
