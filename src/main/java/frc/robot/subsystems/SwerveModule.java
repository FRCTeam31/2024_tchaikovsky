package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.DriveMap;
import prime.utilities.CTREConverter;

public class SwerveModule extends SubsystemBase {

  private int id;
  private double _lastOutput;
  private TalonFX mSteeringMotor;
  private TalonFX mDriveMotor;
  private CANcoder mEncoder;

  private int mEncoderOffset;
  // private SupplyCurrentLimitConfiguration mSupplyCurrentConfig = new SupplyCurrentLimitConfiguration(
  //   true,
  //   50,
  //   80,
  //   0.15
  // );

  private PIDController _steeringPidController;

  public SwerveModule(
    int driveMotorId,
    int steeringMotorId,
    int encoderId,
    int encoderAbsoluteOffset,
    boolean driveInverted
  ) {
    _steeringPidController =
      new PIDController(
        DriveMap.kSteeringPidConstants.kP,
        DriveMap.kSteeringPidConstants.kI,
        DriveMap.kD_min,
        0.020
      );
    id = driveMotorId;
    mEncoderOffset = encoderAbsoluteOffset;

    // Set up the steering motor
    setupSteeringMotor(steeringMotorId);

    // Set up the drive motor
    setupDriveMotor(driveMotorId, driveInverted);

    // Set up our encoder
    mEncoder = new CANcoder(encoderId);
    mEncoder.clearStickyFaults();
    mEncoder.getConfigurator().apply(new CANcoderConfiguration());
    mEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // Create a PID controller to calculate steering motor output
    _steeringPidController.enableContinuousInput(-180, 180);
    _steeringPidController.setTolerance(1);
  }

  private void setupSteeringMotor(int steeringId) {
    mSteeringMotor = new TalonFX(steeringId);

    mSteeringMotor.getConfigurator().apply(new TalonFXConfiguration());
    mSteeringMotor.clearStickyFaults();
    mSteeringMotor.setNeutralMode(NeutralModeValue.Brake);
    // Counter Clockwise Inversion
    mSteeringMotor.setInverted(false);
    CurrentLimitsConfigs mSteeringCurrentLimit = setSupplyCurrentLimit(
      true,
      50,
      80,
      0.15
    );
    mSteeringMotor.getConfigurator().apply(mSteeringCurrentLimit);
  }

  public void setupDriveMotor(int driveMotorId, boolean driveInverted) {
    mDriveMotor = new TalonFX(driveMotorId);
    mSteeringMotor.getConfigurator().apply(new TalonFXConfiguration());
    mDriveMotor.clearStickyFaults();
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.slot0.kP = 0.15;
    driveMotorConfig.slot0.kF = 0.01;

    driveMotorConfig.slot0.allowableClosedloopError = 250;
    mDriveMotor.configAllSettings(driveMotorConfig);
    mDriveMotor.setNeutralMode(NeutralMode.Brake);
    mDriveMotor.setInverted(
      driveInverted
        ? TalonFXInvertType.CounterClockwise
        : TalonFXInvertType.Clockwise
    );
    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // The integrated sensor in the
    // Falcon is the falcon's encoder
    mDriveMotor.configClosedloopRamp(0.5);
    mDriveMotor.configOpenloopRamp(0.5);
  }

  /**
   * Reports pertinent data to the dashboard
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Swerve/" + id + "/Heading",
      getOffsetAbsoluteRotation2d().getDegrees()
    );
    SmartDashboard.putNumber(
      "Swerve/" + id + "/Drive vel",
      getVelocityMetersPerSecond()
    );
    // SmartDashboard.putNumber("Drive vel =>", mDriveMotor.getClosedLoopTarget(0));
    SmartDashboard.putNumber(
      "Swerve/" + id + "/Drive output V",
      mDriveMotor.getMotorOutputVoltage()
    );
    SmartDashboard.putNumber(
      "Swerve/" + id + "/Drive output %",
      mDriveMotor.getMotorOutputPercent()
    );
    SmartDashboard.putNumber(
      "Swerve/" + id + "/Steering output",
      mSteeringMotor.get()
    );
    SmartDashboard.putNumber(
      "Swerve/" + id + "/PID error",
      _steeringPidController.getPositionError()
    );
    SmartDashboard.putNumber("Swerve/" + id + "/PID last output", _lastOutput);
  }

  /**
   * Gets the cumulative SwerveModulePosition of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      CTREConverter.falconToMeters(
        mDriveMotor.getSelectedSensorPosition(),
        DriveMap.kDriveWheelCircumference,
        DriveMap.kDriveGearRatio
      ),
      getOffsetAbsoluteRotation2d()
    );
  }

  /**
   * Sets the setpoint of the steering PID to the new angle provided
   *
   * @param angle the new angle for the module to steer to
   */
  public void setDesiredAngle(Rotation2d angle) {
    var newOutput = _steeringPidController.calculate(
      getMeasurement(),
      angle.getDegrees()
    );
    mSteeringMotor.set(
      ControlMode.PercentOutput,
      MathUtil.clamp(newOutput, -1, 1)
    );
  }

  /**
   * Sets the desired speed of the module in closed-loop velocity mode
   *
   * @param speedMetersPerSecond The desired speed in meters per second
   * @param inHighGear           The desired high/low speed to use
   */
  public void setDesiredSpeed(double speedMetersPerSecond, boolean inHighGear) {
    if (!inHighGear) speedMetersPerSecond *= DriveMap.kLowGearCoefficient;

    mDriveMotor.set(
      ControlMode.Velocity,
      CTREConverter.MPSToFalcon(
        speedMetersPerSecond,
        DriveMap.kDriveWheelCircumference,
        DriveMap.kDriveGearRatio
      )
    );
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(
    SwerveModuleState desiredState,
    boolean inHighGear
  ) {
    // Optimize the state to avoid turning wheels further than 90 degrees
    var encoderRotation = getOffsetAbsoluteRotation2d();
    desiredState = SwerveModuleState.optimize(desiredState, encoderRotation);
    setDesiredSpeed(desiredState.speedMetersPerSecond, inHighGear);
    setDesiredAngle(desiredState.angle);
  }

  /**
   * Sets the encoder position to a new value
   *
   * @param newPosition the new position of the encoder
   */
  public void setEncoderPosition(double newPosition) {
    mEncoder.setPosition(newPosition);
  }

  /**
   * Stops both of the module's motors
   */
  public void stopMotors() {
    mDriveMotor.stopMotor();
    mSteeringMotor.stopMotor();
  }

  /**
   * Gets the velocity of the drive motor in meters per second
   */
  public double getVelocityMetersPerSecond() {
    return CTREConverter.falconToMPS(
      mDriveMotor.getSelectedSensorVelocity(),
      DriveMap.kDriveWheelCircumference,
      DriveMap.kDriveGearRatio
    );
  }

  /**
   * Gets the absolute position of the encoder in degrees
   */
  public double getEncoderAbsolutePosition() {
    return mEncoder.getAbsolutePosition();
  }

  /**
   * Gets the PIDSubsystem measurement term (absolute degrees)
   */
  protected double getMeasurement() {
    return getOffsetAbsoluteRotation2d().getDegrees();
  }

  /**
   * Gets the absolute Rotation2d describing the heading of the module
   */
  private Rotation2d getOffsetAbsoluteRotation2d() {
    return Rotation2d.fromDegrees(
      getEncoderAbsolutePosition() - mEncoderOffset
    );
  }

  private CurrentLimitsConfigs setSupplyCurrentLimit(
    boolean enable,
    double currentLimit,
    double thresholdLimit,
    double thresholdTime
  ) {
    CurrentLimitsConfigs mSupplyCurrentConfig = new CurrentLimitsConfigs();
    mSupplyCurrentConfig.withSupplyCurrentLimitEnable(enable);
    mSupplyCurrentConfig.withSupplyCurrentLimit(currentLimit);
    mSupplyCurrentConfig.withSupplyCurrentThreshold(thresholdLimit);
    mSupplyCurrentConfig.withSupplyTimeThreshold(thresholdTime);

    return mSupplyCurrentConfig;
  }
}
