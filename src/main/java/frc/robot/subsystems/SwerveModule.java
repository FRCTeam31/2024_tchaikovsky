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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.SwerveModuleConfig;
import java.util.Map;
import prime.control.PrimePIDConstants;
import prime.movers.LazyCANSparkMax;
import prime.utilities.CTREConverter;

public class SwerveModule extends SubsystemBase implements AutoCloseable {

  private SwerveModuleConfig m_config;

  // Shuffleboard configuration
  private ShuffleboardTab d_moduleTab;
  private GenericEntry d_driveVelocityEntry;
  private GenericEntry d_driveVoltageEntry;
  private GenericEntry d_moduleHeadingEntry;

  // Devices
  private LazyCANSparkMax m_SteeringMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_encoder;
  private PIDController m_steeringPidController;

  /* Start at velocity 0, no feed forward, use slot 0 */
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
    PrimePIDConstants drivePID,
    PrimePIDConstants steeringPID
  ) {
    m_config = moduleConfig;
    setName(m_config.ModuleName);

    // Set up the shuffleboard tab
    setupDashboard();

    // Set up the steering motor
    setupSteeringMotor(steeringPID);

    // Set up the drive motor
    setupDriveMotor(drivePID);

    // Set up our encoder
    setupCanCoder();
  }

  /**
   * Sets up the shuffleboard tab for the module and the simple entries
   */
  private void setupDashboard() {
    d_moduleTab = Shuffleboard.getTab(getName() + " Module");
    d_driveVelocityEntry =
      d_moduleTab
        .add("Velocity (MPS)", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 20))
        .getEntry();
    d_driveVoltageEntry =
      d_moduleTab
        .add("Voltage (V)", 0)
        .withWidget(BuiltInWidgets.kVoltageView)
        .getEntry();
    d_moduleHeadingEntry =
      d_moduleTab
        .add("Heading (Degrees)", 0)
        .withWidget(BuiltInWidgets.kGyro)
        .withProperties(Map.of("major tick spacing", 15, "starting angle", 0))
        .getEntry();
  }

  /**
   * Sets up the steering motor and PID controller
   */
  private void setupSteeringMotor(PrimePIDConstants pid) {
    m_SteeringMotor =
      new LazyCANSparkMax(m_config.SteeringMotorCanId, MotorType.kBrushless);
    m_SteeringMotor.restoreFactoryDefaults();

    m_SteeringMotor.setSmartCurrentLimit(100, 80);
    m_SteeringMotor.clearFaults();
    m_SteeringMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_SteeringMotor.setInverted(m_config.SteerInverted); // CCW inversion

    // Create a PID controller to calculate steering motor output
    m_steeringPidController = new PIDController(pid.kP, pid.kI, pid.kD, 0.020);
    m_steeringPidController.enableContinuousInput(0, 1); // 0 to 1 rotation
    m_steeringPidController.setTolerance((1 / 360.0) * 5); // 5 degrees in units of rotations
    d_moduleTab
      .add("Steering PID", m_steeringPidController)
      .withWidget(BuiltInWidgets.kPIDController);
  }

  /**
   * Sets up the drive motor
   */
  public void setupDriveMotor(PrimePIDConstants pid) {
    m_driveMotor = new TalonFX(m_config.DriveMotorCanId);
    m_driveMotor.clearStickyFaults();
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration()); // Reset to factory default

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

    // Set the PID values for slot 0
    driveMotorConfig.Slot0 =
      new Slot0Configs()
        .withKP(pid.kP)
        .withKI(pid.kI)
        .withKD(pid.kD)
        .withKV(pid.kV);

    // Set the voltage limits
    driveMotorConfig.Voltage.PeakForwardVoltage = 12;
    driveMotorConfig.Voltage.PeakReverseVoltage = -12;

    // Set the current limits
    driveMotorConfig.withCurrentLimits(m_config.DriveCurrentLimitConfiguration);

    // Set the ramp rates
    driveMotorConfig.withClosedLoopRamps(
      m_config.DriveClosedLoopRampConfiguration
    );

    // Apply the configuration
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

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
      getVelocityMetersPerSecond(),
      getEncoderHeadingRotation2d()
    );
  }

  /**
   * Sets the setpoint of the steering PID to the new angle provided
   *
   * @param angle the new angle for the module to steer to
   */
  public void setDesiredAngle(Rotation2d angle) {
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
        .withAcceleration(speedRotationsPerSecond / 2)
    );
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // desiredState = optimize(desiredState);
    if (m_steeringPidController.atSetpoint()) {
      setDesiredSpeed(
        CTREConverter.metersToRotations(
          desiredState.speedMetersPerSecond,
          m_config.DriveWheelCircumferenceMeters,
          m_config.DriveGearRatio
        )
      );
    }

    setDesiredAngle(desiredState.angle);
  }

  public SwerveModuleState optimize(SwerveModuleState desiredState) {
    double actualAngle = getEncoderHeading();
    double desiredAngle = desiredState.angle.getDegrees();
    double inputInv = (desiredAngle + 180) % 360;
    double distNonInv = Math.abs(actualAngle - desiredAngle);
    double distToInv = Math.abs(actualAngle - inputInv);
    SwerveModuleState optimizedState;

    if (distToInv < distNonInv) {
      optimizedState =
        new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          Rotation2d.fromDegrees(distToInv)
        );
    } else {
      optimizedState =
        new SwerveModuleState(
          desiredState.speedMetersPerSecond,
          Rotation2d.fromDegrees(distNonInv)
        );
    }

    return optimizedState;
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
    var rawHeading = m_encoder.getAbsolutePosition().getValueAsDouble();
    // TODO: figure out why adjustment is necessary

    // return DriverStation.isAutonomous()
    //   ? Rotation2d
    //     .fromRotations(rawHeading)
    //     .rotateBy(Rotation2d.fromDegrees(180))
    //     .getRotations()
    //   : Rotation2d
    //     .fromRotations(rawHeading)
    //     .rotateBy(Rotation2d.fromDegrees(90))
    //     .getRotations();

    return rawHeading;
  }

  /**
   * Gets the encoder heading as a Rotation2d
   */
  protected Rotation2d getEncoderHeadingRotation2d() {
    return Rotation2d.fromRotations(getEncoderHeading());
  }

  /**
   * Updates dashboard data
   */
  @Override
  public void periodic() {
    d_driveVelocityEntry.setDouble(getModuleState().speedMetersPerSecond);
    d_driveVoltageEntry.setDouble(
      m_driveMotor.getMotorVoltage().getValueAsDouble()
    );
    d_moduleHeadingEntry.setDouble(getEncoderHeadingRotation2d().getDegrees());
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
