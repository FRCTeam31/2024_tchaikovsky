package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {

  private RobotConfig m_config;

  // Gyro and Kinematics
  public Pigeon2 m_gyro;
  public SwerveDriveKinematics m_kinematics;
  private boolean m_inHighGear = true;

  // Swerve Modules, in CCW order from FL to FR
  SwerveModule m_frontLeftModule, m_rearLeftModule, m_rearRightModule, m_frontRightModule;
  public SwerveModule[] m_swerveModules;
  public SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[4];

  // Odometry
  SwerveDriveOdometry m_odometry;
  public Field2d m_field;

  // Snap to Gyro Angle PID
  public double m_lastSnapToCalculatedPIDOutput;
  public boolean m_snapToGyroEnabled = false;
  public double m_lastRotationRadians = 0;
  public PIDController m_snapToRotationController;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(RobotConfig config) {
    setName("Drivetrain");
    m_config = config;
    m_gyro = new Pigeon2(config.Drivetrain.PigeonId);
    m_kinematics =
      new SwerveDriveKinematics(
        // in CCW order from FL to FR
        m_config.FrontLeftSwerveModuleConfig.ModuleLocation,
        m_config.RearLeftSwerveModuleConfig.ModuleLocation,
        m_config.RearRightSwerveModuleConfig.ModuleLocation,
        m_config.FrontRightSwerveModuleConfig.ModuleLocation
      );

    // Create swerve modules and ODO
    createSwerveModulesAndOdometry();

    // Configure field
    m_field = new Field2d();
    SmartDashboard.putData(getName() + "/Field", m_field);
    // Configure snap-to PID
    m_snapToRotationController =
      new PIDController(
        m_config.Drivetrain.SnapToPidConstants.kP,
        m_config.Drivetrain.SnapToPidConstants.kI,
        m_config.Drivetrain.SnapToPidConstants.kD,
        0.02
      );
    m_snapToRotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_snapToRotationController.setSetpoint(0);

    m_inHighGear = m_config.Drivetrain.StartInHighGear;

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(0.1, 0, 0), // Translation PID constants
        new PIDConstants(0, 0, 0), // Rotation PID constants        0.7
        m_config.Drivetrain.MaxSpeedMetersPerSecond, // Max module speed, in m/s
        m_config.Drivetrain.WheelBaseCircumferenceMeters / Math.PI / 2, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Creates the swerve modules and starts odometry
   */
  private void createSwerveModulesAndOdometry() {
    m_frontLeftModule = new SwerveModule(m_config.FrontLeftSwerveModuleConfig);
    m_frontRightModule =
      new SwerveModule(m_config.FrontRightSwerveModuleConfig);
    m_rearLeftModule = new SwerveModule(m_config.RearLeftSwerveModuleConfig);
    m_rearRightModule = new SwerveModule(m_config.RearRightSwerveModuleConfig);

    m_odometry =
      new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] { // in CCW order from FL to FR
          m_frontLeftModule.getPosition(),
          m_rearLeftModule.getPosition(),
          m_rearRightModule.getPosition(),
          m_frontRightModule.getPosition(),
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      );

    // in CCW order from FL to FR
    m_swerveModules =
      new SwerveModule[] {
        m_frontLeftModule,
        m_rearLeftModule,
        m_rearRightModule,
        m_frontLeftModule,
      };
  }

  /**
   * Resets the gyro
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Drives using cartesian speeds + a rotation speed
   *
   * @param strafeXMetersPerSecond
   * @param forwardMetersPerSecond
   * @param rotationRadiansPerSecond
   * @param fieldRelative
   */
  public void driveFromCartesianSpeeds(
    double strafeXMetersPerSecond,
    double forwardMetersPerSecond,
    double rotationRadiansPerSecond,
    boolean fieldRelative
  ) {
    // Invert Y axis
    // forwardMetersPerSecond *= -1;

    ChassisSpeeds desiredChassisSpeeds;

    if (!m_inHighGear) {
      strafeXMetersPerSecond *= m_config.Drivetrain.LowGearScalar;
      forwardMetersPerSecond *= m_config.Drivetrain.LowGearScalar;
      rotationRadiansPerSecond *= m_config.Drivetrain.LowGearScalar;
    }

    if (fieldRelative) {
      desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          strafeXMetersPerSecond,
          forwardMetersPerSecond,
          rotationRadiansPerSecond,
          m_gyro.getRotation2d()
        );
    } else {
      desiredChassisSpeeds =
        new ChassisSpeeds(
          strafeXMetersPerSecond,
          forwardMetersPerSecond,
          rotationRadiansPerSecond
        );
    }

    // _lastSnapToCalculatedPIDOutput =
    // _snapToRotationController.calculate(Gyro.getRotation2d().getRadians(),
    // desiredChassisSpeeds.omegaRadiansPerSecond);

    // if (m_snapToGyroEnabled) {
    //   m_lastSnapToCalculatedPIDOutput =
    //     m_snapToRotationController.calculate(
    //       MathUtil.angleModulus(m_gyro.getRotation2d().getRadians())
    //     );
    //   desiredChassisSpeeds.omegaRadiansPerSecond =
    //     -1 * m_lastSnapToCalculatedPIDOutput;
    // }

    m_lastRotationRadians = desiredChassisSpeeds.omegaRadiansPerSecond;

    drive(desiredChassisSpeeds);
  }

  /**
   * Drives using an input ChassisSpeeds
   *
   * @param desiredChassisSpeeds
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds) {
    SmartDashboard.putNumber(
      "Drive/DesiredSpeedX",
      desiredChassisSpeeds.vxMetersPerSecond
    );
    SmartDashboard.putNumber(
      "Drive/DesiredSpeedY",
      desiredChassisSpeeds.vyMetersPerSecond
    );
    SmartDashboard.putNumber(
      "Drive/DesiredSpeedRotation",
      desiredChassisSpeeds.omegaRadiansPerSecond
    );

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
      desiredChassisSpeeds
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      m_config.Drivetrain.MaxSpeedMetersPerSecond
    );

    drive(swerveModuleStates);
  }

  /**
   * Feeds the swerve modules each a desired state.
   *
   * @param swerveModuleStates The new states of the modules in CCW order from FL
   *                           to FR
   */
  public void drive(SwerveModuleState[] swerveModuleStates) {
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[1]);
    m_rearRightModule.setDesiredState(swerveModuleStates[2]);
    m_frontRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Gets the current pose of the drivetrain from odometry
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the position of odometry to the current position, minus 90
   */
  public void resetOdometry(Pose2d pose) {
    m_swerveModulePositions[0] = m_frontLeftModule.getPosition();
    m_swerveModulePositions[1] = m_rearLeftModule.getPosition();
    m_swerveModulePositions[2] = m_rearRightModule.getPosition();
    m_swerveModulePositions[3] = m_frontRightModule.getPosition();

    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      m_swerveModulePositions,
      pose
    );
  }

  /**
   * Gets the direction the robot is facing in degrees, CCW+
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Sets the virtual gearbox shifter
   */
  public void setShift(boolean inHighGear) {
    m_inHighGear = inHighGear;
  }

  /**
   * Toggles the virtual gearbox shifter
   */
  public void toggleShifter() {
    m_inHighGear = !m_inHighGear;
  }

  /**
   * Stops all drivetrain motors
   */
  public void stopMotors() {
    m_frontLeftModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
    m_frontRightModule.stopMotors();
  }

  /**
   * Sets the modules all to a single heading
   */
  public void setWheelAngles(Rotation2d angle) {
    m_frontLeftModule.setDesiredAngle(angle);
    m_rearLeftModule.setDesiredAngle(angle);
    m_rearRightModule.setDesiredAngle(angle);
    m_frontRightModule.setDesiredAngle(angle);
  }

  /**
   * Gets the module positions as an array ordered in standard CCW order
   *
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
      m_frontRightModule.getPosition(),
    };
  }

  public void enableSnapToGyroControl() {
    m_snapToGyroEnabled = true;
  }

  public void disableSnapToGyroControl() {
    m_snapToGyroEnabled = false;
  }

  public void toggleSnapToGyroControl() {
    m_snapToGyroEnabled = !m_snapToGyroEnabled;
    m_snapToRotationController.setSetpoint(0);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
      m_frontLeftModule.getModuleState(),
      m_rearLeftModule.getModuleState(),
      m_rearRightModule.getModuleState(),
      m_frontRightModule.getModuleState()
    );
  }

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Update odometry
    var gyroAngle = m_gyro.getRotation2d();
    SmartDashboard.putNumber("Drive/GyroHeading", m_gyro.getAngle() % 360);
    var robotPose = m_odometry.update(gyroAngle, getModulePositions());
    m_field.setRobotPose(robotPose);

    SmartDashboard.putNumber(
      "Drive/SnapTo/PID error",
      Math.toDegrees(m_snapToRotationController.getPositionError())
    );
    SmartDashboard.putNumber(
      "Drive/SnapTo/PID last output",
      m_lastSnapToCalculatedPIDOutput
    );
    SmartDashboard.putNumber(
      "Drive/SnapTo/Rotation Radians",
      m_lastRotationRadians
    );
    SmartDashboard.putNumber(
      "Drive/SnapTo/ setpoint",
      Math.toDegrees(m_snapToRotationController.getSetpoint())
    );
    SmartDashboard.putBoolean("Drive/SnapTo/ enabled?", m_snapToGyroEnabled);
    SmartDashboard.putNumber(
      "Drive/SnapTo/Current GYro angle",
      Math.toDegrees(MathUtil.angleModulus(m_gyro.getRotation2d().getRadians()))
    );
    SmartDashboard.putBoolean("Drive/ in high gear?", m_inHighGear);
  }

  //#region Commands

  /**
   * Creates a command that drives the robot using the default controls
   */
  public Command defaultDriveCommand(
    DoubleSupplier ySupplier,
    DoubleSupplier xSupplier,
    DoubleSupplier rotationSupplier,
    boolean fieldRelative
  ) {
    return this.run(() -> {
        var strafeX = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.15);
        var forwardY = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.15);
        var rotation = MathUtil.applyDeadband(
          rotationSupplier.getAsDouble(),
          0.1
        );

        strafeX *= m_config.Drivetrain.MaxSpeedMetersPerSecond;
        forwardY *= m_config.Drivetrain.MaxSpeedMetersPerSecond;
        rotation *= m_config.Drivetrain.MaxAngularSpeedRadians;

        driveFromCartesianSpeeds(-strafeX, forwardY, rotation, fieldRelative);
      });
  }

  /**
   * Creates a command that resets the gyro
   */
  public Command resetGyroCommand() {
    return this.runOnce(() -> resetGyro());
  }

  /**
   * Creates a command that resets the odometry
   */
  public Command resetOdometryCommand(Pose2d pose) {
    return this.runOnce(() -> resetOdometry(pose));
  }

  /**
   * Creates a command that toggles the shifter
   */
  public Command toggleShifterCommand() {
    return this.runOnce(() -> toggleShifter());
  }

  /**
   * Creates a command that sets the wheel angles to a single angle
   */
  public Command setWheelAnglesCommand(Rotation2d angle) {
    return this.runOnce(() -> setWheelAngles(angle));
  }

  /**
   * Creates a command that enables snap to gyro control
   */
  public Command enableSnapToGyroControlCommand() {
    return this.runOnce(() -> enableSnapToGyroControl());
  }

  /**
   * Creates a command that toggles snap to gyro control
   */
  public Command toggleSnapToAngleCommand() {
    return this.runOnce(() -> {
        toggleSnapToGyroControl();
      });
  }

  /**
   * Creates a command that drives with snap to gyro control
   */
  public Command driveWithSnapToAngleCommand(double angle) {
    return this.runOnce(() -> {
        enableSnapToGyroControl();
        m_snapToRotationController.setSetpoint(angle);
      });
  }
  //#endregion
}
