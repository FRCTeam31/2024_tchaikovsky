package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class JointSubsystem {

  // Runs the Shooter and Intake to shoot a Note
  public Command RunShooterCommand(
    DoubleSupplier speed,
    Shooter shooter,
    Intake intake
  ) {
    return Commands.run(() -> {
      intake.runIntake(-speed.getAsDouble());
      shooter.runShooter(speed.getAsDouble());
    });
  }
}
