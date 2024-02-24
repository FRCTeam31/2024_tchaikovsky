package frc.robot.config;

import prime.control.PrimePIDConstants;

public class IntakeConfig {

  public int RollersCanId;
  public int NeoLeftCanId;
  public int NeoRightCanId;

  public boolean RollersInverted;
  public boolean NeoLeftInverted;
  public boolean NeoRightInverted;

  public PrimePIDConstants IntakeAnglePid;
  public double PositionDelta;

  public IntakeConfig(
    int rollersCanId,
    int neoLeftCanId,
    int neoRightCanId,
    boolean rollersInverted,
    boolean neoLeftInverted,
    boolean neoRightInverted,
    PrimePIDConstants intakeAnglePid,
    double positionDelta
  ) {
    RollersCanId = rollersCanId;
    NeoLeftCanId = neoLeftCanId;
    NeoRightCanId = neoRightCanId;
    RollersInverted = rollersInverted;
    NeoLeftInverted = neoLeftInverted;
    NeoRightInverted = neoRightInverted;
    IntakeAnglePid = intakeAnglePid;
    PositionDelta = positionDelta;
  }
}
