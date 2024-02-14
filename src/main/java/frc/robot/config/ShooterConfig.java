package frc.robot.config;

public class ShooterConfig {

  public int TalonFXCanID;
  public int VictorSPXCanID;

  public boolean TalonFXInverted;
  public boolean VictorSPXInverted;

  public int LeftLinearActuatorControlChannel;
  public int LeftLinearActuatorAnalogChannel;
  public int RightLinearActuatorControlChannel;
  public int RightLinearActuatorAnalogChannel;

  public int NoteDetectorDIOChannel;

  public ShooterConfig(
    int shooterTalonFXCanID,
    int shooterVictorSPXCanID,
    boolean shooterTalonFXInverted,
    boolean shooterVictorSPXInverted,
    int leftLinearActuatorControlChannel,
    int leftLinearActuatorAnalogChannel,
    int rightLinearActuatorControlChannel,
    int rightLinearActuatorAnalogChannel,
    int noteDetectorDIOChannel
  ) {
    TalonFXCanID = shooterTalonFXCanID;
    VictorSPXCanID = shooterVictorSPXCanID;

    TalonFXInverted = shooterTalonFXInverted;
    VictorSPXInverted = shooterVictorSPXInverted;

    LeftLinearActuatorControlChannel = leftLinearActuatorControlChannel;
    LeftLinearActuatorAnalogChannel = leftLinearActuatorAnalogChannel;
    RightLinearActuatorControlChannel = rightLinearActuatorControlChannel;
    RightLinearActuatorAnalogChannel = rightLinearActuatorAnalogChannel;

    NoteDetectorDIOChannel = noteDetectorDIOChannel;
  }
}
