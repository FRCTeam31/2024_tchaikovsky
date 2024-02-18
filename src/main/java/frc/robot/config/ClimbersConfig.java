package frc.robot.config;

public class ClimbersConfig {

  public int VictorSPXLeftCanID;
  public int VictorSPXRightCanID;

  public boolean LeftInverted;
  public boolean RightInverted;

  public double ClimbersSpeed;

  public int LeftLimitSwitchDIOChannel;
  public int RightLimitSwitchDIOChannel;

  public ClimbersConfig(
    int victorSPXLeftCanID,
    int victorSPXRightCanID,
    boolean leftInverted,
    boolean rightInverted,
    double climberSpeed,
    int leftLimitSwitchDIOChannel,
    int rightLimitSwitchDIOChannel
  ) {
    VictorSPXLeftCanID = victorSPXLeftCanID;
    VictorSPXRightCanID = victorSPXRightCanID;
    LeftInverted = leftInverted;
    RightInverted = rightInverted;
    ClimbersSpeed = climberSpeed;
    LeftLimitSwitchDIOChannel = leftLimitSwitchDIOChannel;
    RightLimitSwitchDIOChannel = rightLimitSwitchDIOChannel;
  }
}
