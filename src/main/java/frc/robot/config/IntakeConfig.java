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

  public int TopLimitSwitchChannel;
  public int BottomLimitSwitchChannel;

  /**
   * Creates a new instance of IntakeConfig with default values
   */
  public IntakeConfig() {
    RollersCanId = 16;
    NeoLeftCanId = 15;
    NeoRightCanId = 14;
    RollersInverted = false;
    NeoLeftInverted = false;
    NeoRightInverted = true;
    IntakeAnglePid = new PrimePIDConstants(0.05, 0, 0);
    PositionDelta = 49;
    TopLimitSwitchChannel = 4;
    BottomLimitSwitchChannel = 5;
  }
}
