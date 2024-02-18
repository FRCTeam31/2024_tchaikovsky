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
  public double PositionMinimum;
  public double PositionMaximum;

  public IntakeConfig(
    int rollersCanId,
    int neoLeftCanId,
    int neoRightCanId,
    boolean rollersInverted,
    boolean neoLeftInverted,
    boolean neoRightInverted,
    PrimePIDConstants intakeAnglePid,
    double positionMinimum,
    double positionMaximum
  ) {
    RollersCanId = rollersCanId;
    NeoLeftCanId = neoLeftCanId;
    NeoRightCanId = neoRightCanId;
    RollersInverted = rollersInverted;
    NeoLeftInverted = neoLeftInverted;
    NeoRightInverted = neoRightInverted;
    IntakeAnglePid = intakeAnglePid;
    PositionMinimum = positionMinimum;
    PositionMaximum = positionMaximum;
  }
}
