package prime.control;

import java.util.function.DoubleSupplier;

public class SwerveControlSuppliers {

  public DoubleSupplier Forward;
  public DoubleSupplier Strafe;
  public DoubleSupplier Rotation;

  public SwerveControlSuppliers(
    DoubleSupplier forwardSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier
  ) {
    Forward = forwardSupplier;
    Strafe = strafeSupplier;
    Rotation = rotationSupplier;
  }
}
