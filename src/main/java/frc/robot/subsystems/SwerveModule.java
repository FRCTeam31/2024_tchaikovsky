package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SwerveModuleConfig;
import prime.utilities.CTREConverter;

public class SwerveModule extends SubsystemBase {

  private CANSparkMax m_SteeringMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_encoder;
  private SwerveModuleConfig m_config;

  private PIDController m_steeringPidController;

  public SwerveModule(SwerveModuleConfig moduleConfig) {
    m_config = moduleConfig;
    setName(m_config.ModuleName);

    // Set up the steering motor
    setupSteeringMotor();

    // Set up the drive motor
    setupDriveMotor();

    // Set up our encoder
    setupCanCoder();
  }

  private void setupSteeringMotor() {
    m_SteeringMotor =
      new CANSparkMax(m_config.SteeringMotorCanId, MotorType.kBrushless);
    m_SteeringMotor.restoreFactoryDefaults();

    m_SteeringMotor.setSmartCurrentLimit(100, 80);
    m_SteeringMotor.clearFaults();
    m_SteeringMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_SteeringMotor.setInverted(m_config.SteerInverted); // CCW inversion

    // Create a PID controller to calculate steering motor output
    m_steeringPidController =
      new PIDController(
        m_config.DrivePidConstants.kP,
        m_config.DrivePidConstants.kI,
        m_config.DrivePidConstants.kD,
        0.020
      );
    m_steeringPidController.enableContinuousInput(-180, 180);
    m_steeringPidController.setTolerance(1);
  }

  public void setupDriveMotor() {
    m_driveMotor = new TalonFX(m_config.DriveMotorCanId);
    m_driveMotor.clearStickyFaults();
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.ClosedLoopRamps =
      m_config.DriveClosedLoopRampConfiguration;
    driveMotorConfig.Slot0 = m_config.DriveSlot0Configuration;
    driveMotorConfig.CurrentLimits = m_config.DriveCurrentLimitConfiguration;

    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setInverted(m_config.DriveInverted); // Clockwise Inversion
  }

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
              .withAbsoluteSensorRange(
                AbsoluteSensorRangeValue.Signed_PlusMinusHalf
              )
          )
      );
  }

  /**
   * Reports data to the dashboard
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Heading",
      getAbsoluteRotation2dWithOffset().getDegrees()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Drive vel",
      getVelocityMetersPerSecond()
    );
    // SmartDashboard.putNumber("Drive vel =>", mDriveMotor.getClosedLoopTarget(0));
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Drive output V",
      m_driveMotor.getMotorVoltage().getValueAsDouble()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Drive output",
      m_driveMotor.get()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Steering output",
      m_SteeringMotor.get()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/PID error",
      m_steeringPidController.getPositionError()
    );
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
      getAbsoluteRotation2dWithOffset()
    );
  }

  /**
   * Sets the setpoint of the steering PID to the new angle provided
   *
   * @param angle the new angle for the module to steer to
   */
  public void setDesiredAngle(Rotation2d angle) {
    // TODO: Rewrite this to use CANSparkMax closed-loop position control
    var newOutput = m_steeringPidController.calculate(
      getMeasurement(),
      angle.getDegrees()
    );
    m_SteeringMotor.set(MathUtil.clamp(newOutput, -1, 1));
  }

  /**
   * Sets the desired speed of the module in closed-loop velocity mode
   *
   * @param speedMetersPerSecond The desired speed in meters per second
   */
  public void setDesiredSpeed(double speedMetersPerSecond) {
    // TODO: Rewrite this to use velocity API in units of rotations per second
    m_driveMotor.set(
      CTREConverter.MPSToFalconTicks(
        speedMetersPerSecond,
        m_config.DriveWheelCircumferenceMeters,
        m_config.DriveGearRatio
      )
    );
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid turning wheels further than 90 degrees
    var encoderRotation = getAbsoluteRotation2dWithOffset();
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);
    setDesiredSpeed(desiredState.speedMetersPerSecond);
    setDesiredAngle(desiredState.angle);
  }

  /**
   * Sets the encoder position to a new value
   *
   * @param newPosition the new position of the encoder
   */
  public void setEncoderPosition(double newPosition) {
    m_encoder.setPosition(newPosition);
  }

  /**
   * Stops both of the module's motors
   */
  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_SteeringMotor.stopMotor();
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

  /**
   * Gets the absolute position of the encoder in degrees
   */
  public double getEncoderAbsoluteRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Gets the PIDSubsystem measurement term (absolute degrees)
   */
  protected double getMeasurement() {
    return getAbsoluteRotation2dWithOffset().getDegrees();
  }

  /**
   * Gets the absolute Rotation2d describing the heading of the module
   */
  private Rotation2d getAbsoluteRotation2dWithOffset() {
    return Rotation2d.fromRotations(
      getEncoderAbsoluteRotations() - (1 / m_config.StartingOffset)
    );
  }
}
