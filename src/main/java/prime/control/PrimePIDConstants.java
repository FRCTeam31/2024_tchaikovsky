package prime.control;

public class PrimePIDConstants {

  public double kP;
  public double kI;
  public double kD;
  public double kF;
  public double kV;
  public double kA;
  public double kS;

  public PrimePIDConstants(
    double kP,
    double kI,
    double kD,
    double kF,
    double kV,
    double kA,
    double kS
  ) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kV = kV;
    this.kA = kA;
    this.kS = kS;
  }

  public PrimePIDConstants() {
    this(0, 0, 0, 0, 0, 0, 0);
  }

  public PrimePIDConstants(double kP, double kI, double kD) {
    this(kP, kI, kD, 0, 0, 0, 0);
  }

  public PrimePIDConstants(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, 0, 0, 0);
  }

  public PrimePIDConstants(
    double kP,
    double kI,
    double kD,
    double kF,
    double kV
  ) {
    this(kP, kI, kD, kF, kV, 0, 0);
  }
}
