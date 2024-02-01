package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.control.Controls;

public class Drivetrain extends SubsystemBase implements AutoCloseable {

  // Container for robot configuration
  private RobotConfig m_config;

  // Shuffleboard configuration
  private ShuffleboardTab d_dashboardTab = Shuffleboard.getTab("Drivetrain");
  private GenericEntry d_snapToEnabledEntry = d_dashboardTab
    .add("SnapTo Enabled", false)
    .getEntry();
  private GenericEntry d_inHighGearEntry = d_dashboardTab
    .add("SnapTo Enabled", false)
    .getEntry();

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

    // Create gyro
    m_gyro = new Pigeon2(config.Drivetrain.PigeonId);
    d_dashboardTab
      .add("Gyro", m_gyro)
      .withWidget(BuiltInWidgets.kGyro)
      .withProperties(Map.of("major tick spacing", 15, "starting angle", 0));

    // Create swerve modules, kinematics, and odometry
    m_kinematics =
      new SwerveDriveKinematics(
        // in CCW order from FL to FR
        m_config.FrontLeftSwerveModule.getModuleLocation(),
        m_config.RearLeftSwerveModule.getModuleLocation(),
        m_config.RearRightSwerveModule.getModuleLocation(),
        m_config.FrontRightSwerveModule.getModuleLocation()
      );
    createSwerveModulesAndOdometry();
    m_inHighGear = config.Drivetrain.StartInHighGear;

    // Configure field
    m_field = new Field2d();
    d_dashboardTab.add("Field", m_field).withWidget(BuiltInWidgets.kField);

    // Configure snap-to PID
    m_snapToRotationController =
      new PIDController(
        m_config.Drivetrain.SnapToPID.kP,
        m_config.Drivetrain.SnapToPID.kI,
        m_config.Drivetrain.SnapToPID.kD,
        0.02
      );
    m_snapToRotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_snapToRotationController.setSetpoint(0);
    d_dashboardTab
      .add("SnapTo PID", m_snapToRotationController)
      .withWidget(BuiltInWidgets.kPIDController);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      m_config.Drivetrain.getHolonomicPathFollowerConfig(),
      () -> false, // Method to determine whether or not to flip the path
      this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Creates the swerve modules and starts odometry
   */
  private void createSwerveModulesAndOdometry() {
    m_frontLeftModule =
      new SwerveModule(
        m_config.FrontLeftSwerveModule,
        m_config.Drivetrain.DrivePID,
        m_config.Drivetrain.SteeringPID
      );
    m_frontRightModule =
      new SwerveModule(
        m_config.FrontRightSwerveModule,
        m_config.Drivetrain.DrivePID,
        m_config.Drivetrain.SteeringPID
      );
    m_rearLeftModule =
      new SwerveModule(
        m_config.RearLeftSwerveModule,
        m_config.Drivetrain.DrivePID,
        m_config.Drivetrain.SteeringPID
      );
    m_rearRightModule =
      new SwerveModule(
        m_config.RearRightSwerveModule,
        m_config.Drivetrain.DrivePID,
        m_config.Drivetrain.SteeringPID
      );

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
   * Drives the robot using cartesian speeds, scaled to the max speed of the robot in meters per second
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

    if (m_snapToGyroEnabled) {
      m_lastSnapToCalculatedPIDOutput =
        m_snapToRotationController.calculate(
          MathUtil.angleModulus(m_gyro.getRotation2d().getRadians())
        );
      desiredChassisSpeeds.omegaRadiansPerSecond =
        -1 * m_lastSnapToCalculatedPIDOutput;
    }

    m_lastRotationRadians = desiredChassisSpeeds.omegaRadiansPerSecond;

    drive(desiredChassisSpeeds);
  }

  /**
   * Drives using an input ChassisSpeeds
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
    var robotPose = m_odometry.update(gyroAngle, getModulePositions());
    m_field.setRobotPose(robotPose);

    // Update shuffleboard entries
    d_snapToEnabledEntry.setBoolean(m_snapToGyroEnabled);
    d_inHighGearEntry.setBoolean(m_inHighGear);
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
        var strafeX = Controls.cubicScaledDeadband(
          xSupplier.getAsDouble(),
          0.15,
          0.1
        );

        var forwardY = Controls.cubicScaledDeadband(
          ySupplier.getAsDouble(),
          0.15,
          0.1
        );
        var rotation = Controls.cubicScaledDeadband(
          rotationSupplier.getAsDouble(),
          0.1,
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

  @Override
  public void close() throws Exception {
    DriverStation.reportWarning(">> Drivetrain closing...", false);

    // close all physical resources
    m_gyro.close();
    m_frontLeftModule.close();
    m_rearLeftModule.close();
    m_rearRightModule.close();
    m_frontRightModule.close();
    m_snapToRotationController.close();

    // release memory resources
    m_kinematics = null;
    m_odometry = null;
    m_field = null;
  }
}
