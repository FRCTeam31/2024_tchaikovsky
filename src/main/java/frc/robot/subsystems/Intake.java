package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  LazyCANSparkMax m_intakeRollerSparkMax;
  LazyCANSparkMax m_intakeAngleSparkMaxLeft;
  LazyCANSparkMax m_intakeAngleSparkMaxRight;
  SparkMaxPIDController m_intakeAnglePid;
  RobotConfig m_RobotConfig;
  RelativeEncoder leftEncoder;

  // Creates a new Intake
  public Intake(RobotConfig robotConfig) {
    m_RobotConfig = robotConfig;
    m_intakeRollerSparkMax =
      new LazyCANSparkMax(
        RobotConfig.m_intakeRollerSparkMaxCanID,
        MotorType.kBrushless
      );

    m_intakeAngleSparkMaxLeft =
      new LazyCANSparkMax(
        RobotConfig.m_intakeAngleSparkMaxLeftCanID,
        MotorType.kBrushless
      );
    m_intakeAngleSparkMaxRight =
      new LazyCANSparkMax(
        RobotConfig.m_intakeAngleSparkMaxRightCanID,
        MotorType.kBrushless
      );
    m_intakeAngleSparkMaxRight.setInverted(true);

    m_intakeAnglePid = m_intakeAngleSparkMaxLeft.getPIDController();

    m_intakeAnglePid.setP(0.1);
    m_intakeAnglePid.setI(0);
    m_intakeAnglePid.setD(0);
    m_intakeAnglePid.setOutputRange(-1, 1);
  }

  // Method for giving the Intake Roller Motor a speed
  public void runIntakeRollers(double speed) {
    m_intakeRollerSparkMax.set(speed);
  }

  public void setIntakeAngle(Rotation2d rotation2d) {
    var desiredRotation = rotation2d.getRotations();
    var desiredRotationWithRatio = desiredRotation * 20;

    setIntakeRotation(desiredRotationWithRatio);
  }

  // Method for setting a rotational setpoint for the intake motors to seek
  public void setIntakeRotation(double rotation) {
    m_intakeAnglePid.setReference(rotation, ControlType.kPosition);

    var motorOutput = m_intakeAngleSparkMaxLeft.get();
    m_intakeAngleSparkMaxRight.set(motorOutput);
  }

  // Command for running the intake to intake a Note
  public Command RunIntakeCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntakeRollers(speed.getAsDouble());
      });
  }

  // Command for changing the angle of the Position
  public Command IntakeAngleCommand(DoubleSupplier speed) {
    return this.run(() -> {
        setIntakeRotation(speed.getAsDouble());
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

  public void close() {
    m_intakeAngleSparkMaxLeft.close();
    m_intakeAngleSparkMaxRight.close();
    m_intakeRollerSparkMax.close();
  }
}
