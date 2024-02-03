package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Intake extends SubsystemBase {

  LazyCANSparkMax m_intakeRollerSparkMax;
  LazyCANSparkMax m_intakeAngleSparkMaxLeft;
  LazyCANSparkMax m_intakeAngleSparkMaxRight;
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
  }

  // Method for giving the Intake Roller Motor a speed
  public void runIntake(double speed) {
    m_intakeRollerSparkMax.set(speed);
  }

  // Method for giving the Intake Angle Motors a speed
  public void intakeAngle(double speed) {
    m_intakeAngleSparkMaxLeft.set(speed);
    m_intakeAngleSparkMaxRight.set(-speed);
  }

  // Command for running the intake to intake a Note
  public Command RunIntakeCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntake(speed.getAsDouble());
      });
  }

  // Command for changing the angle of the Position
  public Command IntakeAngleCommand(DoubleSupplier speed) {
    return this.run(() -> {
        intakeAngle(speed.getAsDouble());
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
}
