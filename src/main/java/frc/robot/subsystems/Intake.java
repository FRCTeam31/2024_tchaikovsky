package frc.robot.subsystems;

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

  public Intake(RobotConfig robotConfig) {
    m_RobotConfig = robotConfig;
  }

  public void runIntake(double speed) {
    m_intakeRollerSparkMax.set(speed);
  }

  public void intakeAngle(double speed) {
    m_intakeAngleSparkMaxLeft.set(speed);
    m_intakeAngleSparkMaxRight.set(-speed);
  }

  public Command RunIntakeCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runIntake(speed.getAsDouble());
      });
  }

  public Command IntakeAngleCommand(DoubleSupplier speed) {
    return this.run(() -> {
        intakeAngle(speed.getAsDouble());
      });
  }
}
