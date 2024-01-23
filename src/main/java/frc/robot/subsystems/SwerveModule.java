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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SwerveModuleConfig;
import prime.movers.LazyCANSparkMax;
import prime.utilities.CTREConverter;

public class SwerveModule extends SubsystemBase implements AutoCloseable {

  private LazyCANSparkMax m_SteeringMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_encoder;
  private SwerveModuleConfig m_config;
  private PIDController m_steeringPidController;

  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(
    0,
    0,
    false,
    0,
    0,
    false,
    false,
    false
  );

  public SwerveModule(
    SwerveModuleConfig moduleConfig,
    double[] drivePID,
    double[] steeringPID
  ) {
    m_config = moduleConfig;
    setName(m_config.ModuleName);

    // Set up the steering motor
    setupSteeringMotor(steeringPID);

    // Set up the drive motor
    setupDriveMotor(drivePID);

    // Set up our encoder
    setupCanCoder();
  }

  /**
   * Sets up the steering motor and PID controller
   */
  private void setupSteeringMotor(double[] pid) {
    m_SteeringMotor =
      new LazyCANSparkMax(m_config.SteeringMotorCanId, MotorType.kBrushless);
    m_SteeringMotor.restoreFactoryDefaults();

    m_SteeringMotor.setSmartCurrentLimit(100, 80);
    m_SteeringMotor.clearFaults();
    m_SteeringMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_SteeringMotor.setInverted(m_config.SteerInverted); // CCW inversion

    // Create a PID controller to calculate steering motor output
    m_steeringPidController = new PIDController(pid[0], pid[1], pid[2], 0.020);
    m_steeringPidController.enableContinuousInput(0, 1);
    m_steeringPidController.setTolerance((1 / 360.0) * 5);
  }

  /**
   * Sets up the drive motor
   */
  public void setupDriveMotor(double[] pid) {
    m_driveMotor = new TalonFX(m_config.DriveMotorCanId);
    m_driveMotor.clearStickyFaults();
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.Slot0 =
      new Slot0Configs().withKP(pid[0]).withKI(pid[1]).withKD(pid[2]);
    driveMotorConfig.Voltage.PeakForwardVoltage = 12;
    driveMotorConfig.Voltage.PeakReverseVoltage = -12;

    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_driveMotor.setInverted(m_config.DriveInverted); // Clockwise Inversion
  }

  /**
   * Sets up the CANCoder
   */
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

  /**
   * Reports data to the dashboard
   */
  @Override
  public void periodic() {
    // SmartDashboard.putNumber(
    //   "Swerve/" + getName() + "/Drive vel",
    //   getVelocityMetersPerSecond()
    // );
    // // SmartDashboard.putNumber("Drive vel =>", mDriveMotor.getClosedLoopTarget(0));
    // SmartDashboard.putNumber(
    //   "Swerve/" + getName() + "/Drive output V",
    //   m_driveMotor.getMotorVoltage().getValueAsDouble()
    // );
    // SmartDashboard.putNumber(
    //   "Swerve/" + getName() + "/Drive output",
    //   m_driveMotor.get()
    // );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Steering output",
      m_SteeringMotor.get()
    );
    SmartDashboard.putBoolean(
      "Swerve/" + getName() + "/PID On-Target",
      m_steeringPidController.atSetpoint()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/PID-error",
      m_steeringPidController.getPositionError()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/PID-setpoint",
      m_steeringPidController.getSetpoint()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/PID-Measurement",
      getEncoderHeading()
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
      getEncoderHeadingRotation2d()
    );
  }

  /**
   * Sets the setpoint of the steering PID to the new angle provided
   *
   * @param angle the new angle for the module to steer to
   */
  public void setDesiredAngle(Rotation2d angle) {
    // TODO: figure out why this is necessary
    angle = angle.rotateBy(Rotation2d.fromDegrees(-90));

    var setpoint = angle.getRotations() % 1;
    if (setpoint < 0) setpoint += 1;

    var newOutput = m_steeringPidController.calculate(
      getEncoderHeading(),
      setpoint
    );

    m_SteeringMotor.set(MathUtil.clamp(newOutput, -1, 1));
  }

  /**
   * Sets the desired speed of the module in closed-loop velocity mode
   *
   * @param speedMetersPerSecond The desired speed in meters per second
   */
  public void setDesiredSpeed(double speedRotationsPerSecond) {
    m_driveMotor.setControl(
      m_voltageVelocity
        .withVelocity(speedRotationsPerSecond)
        .withAcceleration(speedRotationsPerSecond)
    );
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO: Optimize the state to avoid turning wheels further than 90 degrees
    // var encoderRotation = getEncoderHeadingRotation2d();
    // desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);
    // SmartDashboard.putNumber(
    //   "Swerve/" + getName() + "/Optimized Angle",
    //   desiredState.angle.getDegrees()
    // );
    // SmartDashboard.putNumber(
    //   "Swerve/" + getName() + "/Optimized Speed",
    //   desiredState.speedMetersPerSecond
    // );

    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Desired Angle",
      desiredState.angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "Swerve/" + getName() + "/Desired Speed",
      desiredState.speedMetersPerSecond
    );

    setDesiredSpeed(
      CTREConverter.metersToRotations(
        desiredState.speedMetersPerSecond,
        m_config.DriveWheelCircumferenceMeters,
        m_config.DriveGearRatio
      )
    );

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
   * Gets the heading of the encoder in rotations
   */
  public double getEncoderHeading() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Gets the encoder heading as a Rotation2d
   */
  protected Rotation2d getEncoderHeadingRotation2d() {
    return Rotation2d.fromRotations(getEncoderHeading());
  }

  @Override
  public void close() {
    DriverStation.reportWarning(
      ">> Module " + getName() + " closing...",
      false
    );
    m_SteeringMotor.close();
    m_driveMotor.close();
    m_encoder.close();
    m_steeringPidController.close();
  }
}
