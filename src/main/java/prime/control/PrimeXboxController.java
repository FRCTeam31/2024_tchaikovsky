package prime.control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public class PrimeXboxController extends CommandXboxController {

  public PrimeXboxController(int port) {
    super(port);
  }

  public DoubleSupplier getLeftStickYSupplier(
    double deadband,
    double curveWeight
  ) {
    return () ->
      Controls.cubicScaledDeadband(
        getRawAxis(Controls.LEFT_STICK_X),
        deadband,
        curveWeight
      );
  }

  public DoubleSupplier getLeftStickXSupplier(
    double deadband,
    double curveWeight
  ) {
    return () ->
      Controls.cubicScaledDeadband(
        getRawAxis(Controls.LEFT_STICK_X),
        deadband,
        curveWeight
      );
  }

  public DoubleSupplier getRightStickYSupplier(
    double deadband,
    double curveWeight
  ) {
    return () ->
      Controls.cubicScaledDeadband(
        getRawAxis(Controls.RIGHT_STICK_Y),
        deadband,
        curveWeight
      );
  }

  public DoubleSupplier getRightStickXSupplier(
    double deadband,
    double curveWeight
  ) {
    return () ->
      Controls.cubicScaledDeadband(
        getRawAxis(Controls.RIGHT_STICK_X),
        deadband,
        curveWeight
      );
  }

  public DoubleSupplier getTriggerSupplier() {
    return () ->
      getRawAxis(Controls.RIGHT_TRIGGER) - getRawAxis(Controls.LEFT_TRIGGER);
  }
}
