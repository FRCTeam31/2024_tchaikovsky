package frc.robot.config;

public class ShooterConfig {

  public int TalonFXCanID;
  public int VictorSPXCanID;

  public boolean TalonFXInverted;
  public boolean VictorSPXInverted;

  public int LeftLinearActuatorCanID;
  public int LeftLinearActuatorAnalogChannel;
  public int RightLinearActuatorCanID;
  public int RightLinearActuatorAnalogChannel;

  public int NoteDetectorDIOChannel;

  public double MinimumElevation;
  public double MaximumElevation;

  public ShooterConfig(
    int shooterTalonFXCanID,
    int shooterVictorSPXCanID,
    boolean shooterTalonFXInverted,
    boolean shooterVictorSPXInverted,
    int leftLinearActuatorCanID,
    int leftLinearActuatorAnalogChannel,
    int rightLinearActuatorCanID,
    int rightLinearActuatorAnalogChannel,
    int noteDetectorDIOChannel,
    double minimumElevation,
    double maximumElevation
  ) {
    TalonFXCanID = shooterTalonFXCanID;
    VictorSPXCanID = shooterVictorSPXCanID;

    TalonFXInverted = shooterTalonFXInverted;
    VictorSPXInverted = shooterVictorSPXInverted;

    LeftLinearActuatorCanID = leftLinearActuatorCanID;
    LeftLinearActuatorAnalogChannel = leftLinearActuatorAnalogChannel;
    RightLinearActuatorCanID = rightLinearActuatorCanID;
    RightLinearActuatorAnalogChannel = rightLinearActuatorAnalogChannel;
    NoteDetectorDIOChannel = noteDetectorDIOChannel;

    MinimumElevation = minimumElevation;
    MaximumElevation = maximumElevation;
  }
}
