package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SwerveModuleConfig;
import prime.control.PrimePIDConstants;
import prime.movers.LazyCANSparkMax;
import prime.utilities.CTREConverter;

public class SwerveModule extends SubsystemBase {

  private SwerveModuleConfig m_config;

  // Devices
  private LazyCANSparkMax m_SteeringMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_encoder;
  private PIDController m_steeringPidController;

  // Start at velocity 0, no feed forward, use slot 0
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

  public SwerveModule(SwerveModuleConfig moduleConfig, PrimePIDConstants drivePID, PrimePIDConstants steeringPID) {
    m_config = moduleConfig;
    setName(m_config.ModuleName);

    setupSteeringMotor(steeringPID);
    setupDriveMotor(drivePID);
    setupCanCoder();
  }

  //#region Setup methods

  // Sets up the steering motor and PID controller
  private void setupSteeringMotor(PrimePIDConstants pid) {
    m_SteeringMotor = new LazyCANSparkMax(m_config.SteeringMotorCanId, MotorType.kBrushless);
    m_SteeringMotor.restoreFactoryDefaults();

    m_SteeringMotor.setSmartCurrentLimit(100, 80);
    m_SteeringMotor.clearFaults();
    m_SteeringMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_SteeringMotor.setInverted(m_config.SteerInverted); // CCW inversion

    // Create a PID controller to calculate steering motor output
    m_steeringPidController = pid.createPIDController(0.02);
    m_steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
    m_steeringPidController.setTolerance((1 / 360.0) * 2); // 2 degrees in units of rotations
  }

  // Sets up the drive motors
  public void setupDriveMotor(PrimePIDConstants pid) {
    m_driveMotor = new TalonFX(m_config.DriveMotorCanId);
    m_driveMotor.clearStickyFaults();
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration()); // Reset to factory default

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    // Set the PID values for slot 0
    driveMotorConfig.Slot0 =
      new Slot0Configs().withKP(pid.kP).withKI(pid.kI).withKD(pid.kD).withKS(pid.kS).withKV(pid.kV);

    // Set the voltage limits
    driveMotorConfig.Voltage.PeakForwardVoltage = 12;
    driveMotorConfig.Voltage.PeakReverseVoltage = -12;

    // Set the current limits
    driveMotorConfig.withCurrentLimits(m_config.DriveCurrentLimitConfiguration);

    // Set the ramp rates
    driveMotorConfig.withClosedLoopRamps(m_config.DriveClosedLoopRampConfiguration);

    // Apply the configuration
    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setInverted(m_config.DriveInverted); // Clockwise Inversion
  }

  // Sets up the CANCoder
  public void setupCanCoder() {
    m_encoder = new CANcoder(m_config.CANCoderCanId);
    m_encoder.clearStickyFaults();
    m_encoder.getConfigurator().apply(new CANcoderConfiguration());

    // AbsoluteSensorRangeValue
    m_encoder
      .getConfigurator()
      .apply(
        new CANcoderConfiguration()
          .withMagnetSensor(
            new MagnetSensorConfigs()
              .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
              .withMagnetOffset(-m_config.StartingOffset)
          )
      );
  }

  //#endregion

  //#region Control methods

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Set the drive motor to the desired speed
    var speedRotationsPerSecond = CTREConverter.metersToRotations(
      desiredState.speedMetersPerSecond,
      m_config.DriveWheelCircumferenceMeters,
      m_config.DriveGearRatio
    );

    m_driveMotor.setControl(
      m_voltageVelocity.withVelocity(speedRotationsPerSecond) // TODO: evaluate effect of removing ".withAcceleration(speedRotationsPerSecond / 2)"
    );

    // Set the steering motor to the desired angle
    var setpoint = desiredState.angle.getRotations() % 1;
    if (setpoint < 0) setpoint += 1;

    var newOutput = m_steeringPidController.calculate(getEncoderHeading(), setpoint);

    m_SteeringMotor.set(MathUtil.clamp(newOutput, -1, 1));
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the shortest path to drive at the desired angle
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    Rotation2d currentAngle = getEncoderHeadingRotation2d();
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))
      );
    } else {
      return desiredState;
    }
  }

  // Stops both motors within the Module
  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_SteeringMotor.stopMotor();
  }

  /**
   * Gets the cumulative SwerveModulePosition of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      CTREConverter.rotationsToMeters(
        m_driveMotor.getPosition().getValueAsDouble(),
        m_config.DriveWheelCircumferenceMeters,
        m_config.DriveGearRatio
      ),
      getEncoderHeadingRotation2d()
    );
  }

  /**
   * Gets the current state of the module
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getEncoderHeadingRotation2d());
  }

  /**
   * Gets the velocity of the drive motor in meters per second
   */
  public double getVelocityMetersPerSecond() {
    return CTREConverter.rotationsToMeters(
      m_driveMotor.getVelocity().getValueAsDouble(),
      m_config.DriveWheelCircumferenceMeters,
      m_config.DriveGearRatio
    );
  }

  // Gets the heading of the encoder in rotations
  public double getEncoderHeading() {
    return m_encoder.getPosition().getValueAsDouble();
  }

  // Gets the encoder heading as a Rotation2d
  protected Rotation2d getEncoderHeadingRotation2d() {
    return Rotation2d.fromRotations(getEncoderHeading());
  }

  //#endregion

  /**
   * Updates dashboard data
   */
  @Override
  public void periodic() {
    // d_driveVelocityEntry.setDouble(getModuleState().speedMetersPerSecond);
    // d_driveVoltageEntry.setDouble(m_driveMotor.getMotorVoltage().getValueAsDouble());
    // d_moduleHeadingEntry.setDouble(getEncoderHeadingRotation2d().getDegrees());
  }
}
