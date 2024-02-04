package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  RobotConfig m_config;

  LazyCANSparkMax m_rollers;
  LazyCANSparkMax m_neoLeft;
  LazyCANSparkMax m_neoRight;
  SparkMaxPIDController m_intakeAnglePid;

  /**
   * Creates a new Intake subsystem
   * @param robotConfig
   */
  public Intake(RobotConfig robotConfig) {
    m_config = robotConfig;
    m_rollers =
      new LazyCANSparkMax(
        RobotConfig.m_intakeRollerSparkMaxCanID,
        MotorType.kBrushless
      );

    m_neoLeft =
      new LazyCANSparkMax(
        RobotConfig.m_intakeAngleSparkMaxLeftCanID,
        MotorType.kBrushless
      );
    m_neoRight =
      new LazyCANSparkMax(
        RobotConfig.m_intakeAngleSparkMaxRightCanID,
        MotorType.kBrushless
      );
    m_neoRight.setInverted(true);

    m_intakeAnglePid = m_neoLeft.getPIDController();
    m_intakeAnglePid.setP(0.1);
    m_intakeAnglePid.setI(0);
    m_intakeAnglePid.setD(0);
    m_intakeAnglePid.setOutputRange(-1, 1);
  }

  /**
   * Runs intake rollers at a given speed
   * @param speed
   */
  public void runIntakeRollers(double speed) {
    m_rollers.set(speed);
  }

  /**
   * Sets the angle of the intake
   * @param rotation2d
   */
  public void setIntakeAngle(Rotation2d rotation2d) {
    var desiredRotation = rotation2d.getRotations();
    var desiredRotationWithRatio = desiredRotation * 20; // Adjust for gear ratio

    setIntakeRotation(desiredRotationWithRatio);
  }

  /**
   * Sets a rotational setpoint for the intake motors to seek
   * @param rotation the setpoint
   */
  public void setIntakeRotation(double rotation) {
    m_intakeAnglePid.setReference(rotation, ControlType.kPosition);

    var motorOutput = m_neoLeft.get();
    m_neoRight.set(motorOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Intake Left Position",
      m_neoLeft.getEncoder().getPosition()
    );

    SmartDashboard.putNumber(
      "Intake Right Position",
      m_neoRight.getEncoder().getPosition()
    );

    SmartDashboard.putNumber("Intake Roller Speed Magnitude", m_rollers.get());
  }

  //#region Commands

  /**
   * Command for running the rollers
   */
  public Command runRollersCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntakeRollers(speed.getAsDouble());
      });
  }

  /**
   * Command for changing the angle of the intake
   * @param speed
   * @return
   */
  public Command setIntakeRotation(Rotation2d rotation) {
    return this.run(() -> {
        setIntakeRotation(rotation);
      });
  }

  // Command for stopping the Intake Motors
  public Command stopAllMotorsCommand() {
    return this.run(() -> {
        m_neoLeft.stopMotor();
        m_neoRight.stopMotor();
        m_rollers.stopMotor();
      });
  }

  //#endregion

  /**
   * Closes all internal resources
   */
  public void close() {
    m_neoLeft.close();
    m_neoRight.close();
    m_rollers.close();
  }
}
