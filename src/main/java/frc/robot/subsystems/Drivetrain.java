package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import java.util.Map;
import prime.control.Controls;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDSection;
import prime.control.SwerveControlSuppliers;
import prime.movers.IPlannable;

public class Drivetrain extends SubsystemBase implements IPlannable {

  // Container for robot configuration
  private RobotConfig m_config;

  // Shuffleboard configuration
  private ShuffleboardTab d_driveTab = Shuffleboard.getTab("Drivetrain");
  public GenericEntry d_snapToEnabledEntry = d_driveTab
    .add("SnapTo Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0, 0)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry d_currentHeading = d_driveTab
    .add("Current Heading", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(14, 0)
    .withSize(4, 5)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();
  private GenericEntry d_snapAngle = d_driveTab
    .add("SnapTo Angle", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(2, 0)
    .withSize(4, 5)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();

  // Gyro and swerve modules in CCW order from FL to FR
  public Pigeon2 m_gyro;
  private SwerveModule m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;

  // Kinematics, odometry, and field widget
  public Limelight Limelight;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;
  public Field2d m_fieldWidget;

  // Snap to Gyro Angle PID & Lock-on
  public boolean m_lockOnEnabled = false;
  public boolean m_snapToGyroEnabled = false;
  public PIDController m_snapToRotationController;

  private LEDs m_leds;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(RobotConfig config, LEDs leds) {
    setName("Drivetrain");
    m_config = config;

    m_leds = leds;

    // Create gyro
    m_gyro = new Pigeon2(config.Drivetrain.PigeonId);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());

    // Create swerve modules in CCW order from FL to FR
    m_frontLeftModule =
      new SwerveModule(m_config.FrontLeftSwerveModule, m_config.Drivetrain.DrivePID, m_config.Drivetrain.SteeringPID);
    m_frontRightModule =
      new SwerveModule(m_config.FrontRightSwerveModule, m_config.Drivetrain.DrivePID, m_config.Drivetrain.SteeringPID);
    m_rearLeftModule =
      new SwerveModule(m_config.RearLeftSwerveModule, m_config.Drivetrain.DrivePID, m_config.Drivetrain.SteeringPID);
    m_rearRightModule =
      new SwerveModule(m_config.RearRightSwerveModule, m_config.Drivetrain.DrivePID, m_config.Drivetrain.SteeringPID);

    // Create kinematics and odometry
    Limelight = new Limelight(m_config.LimelightPose);
    m_kinematics =
      new SwerveDriveKinematics(
        m_config.FrontLeftSwerveModule.getModuleLocation(),
        m_config.FrontRightSwerveModule.getModuleLocation(),
        m_config.RearLeftSwerveModule.getModuleLocation(),
        m_config.RearRightSwerveModule.getModuleLocation()
      );
    m_poseEstimator =
      new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), getModulePositions(), new Pose2d());

    // Configure field widget and feed it PathPlanner's current path poses
    m_fieldWidget = new Field2d();
    d_driveTab.add("Field", m_fieldWidget).withWidget(BuiltInWidgets.kField).withPosition(6, 0).withSize(8, 5);
    PathPlannerLogging.setLogActivePathCallback(poses -> m_fieldWidget.getObject("path").setPoses(poses));

    // Configure snap-to PID
    m_snapToRotationController = m_config.Drivetrain.SnapToPID.getPIDController(0.02);
    m_snapToRotationController.enableContinuousInput(-Math.PI, Math.PI);
    d_driveTab
      .add("SnapTo PID", m_snapToRotationController)
      .withWidget(BuiltInWidgets.kPIDController)
      .withPosition(0, 2);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      m_config.Drivetrain.getHolonomicPathFollowerConfig(),
      Robot::onRedAlliance, // BooleanSupplier to tell PathPlanner whether or not to flip the path over the Y midline of the field
      this // Reference to this subsystem to set requirements
    );
  }

  //#region Control methods

  // Resets the Gyro
  public void resetGyro() {
    m_gyro.setYaw(Robot.onRedAlliance() ? 0 : 180);
  }

  /**
   * Drives the robot using cartesian speeds, scaled to the max speed of the robot in meters per second
   * @param inputStrafeMPS The sideways motion of the chassis in meters per second
   * @param inputForwardMPS The forward motion of the chassis in meters per second
   * @param inputRotationRadiansPS The rotational motion of the chassis in radians per second
   * @param fieldRelative Whether or not the input is field relative or robot relative
   */
  public void driveFromCartesianSpeeds(
    double inputStrafeMPS,
    double inputForwardMPS,
    double inputRotationRadiansPS,
    boolean fieldRelative
  ) {
    // Build chassis speeds
    ChassisSpeeds desiredChassisSpeeds;
    if (fieldRelative) {
      if (Robot.onBlueAlliance()) {
        // Drive the robot with the driver-relative inputs, converting them to field-relative
        desiredChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
            inputForwardMPS, // Driver is facing field-X+, so use Y component as X+ speed
            -inputStrafeMPS, // Driver is facing field-X+, so use -X component as Y+ speed
            inputRotationRadiansPS, // Rotation is always counter-clockwise
            m_gyro.getRotation2d()
          );
      } else {
        desiredChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
            -inputForwardMPS, // Driver is facing field-X-, so use -Y component as +X speed
            inputStrafeMPS, // Driver is facing field-X-, so use X component as Y+ speed
            inputRotationRadiansPS, // Rotation is always counter-clockwise
            m_gyro.getRotation2d()
          );
      }
    } else {
      // Drive the robot directly with the driver-relative inputs converted to robot-relative speeds
      desiredChassisSpeeds = new ChassisSpeeds(-inputForwardMPS, -inputStrafeMPS, inputRotationRadiansPS);
    }

    drive(desiredChassisSpeeds);
  }

  /**
   * Drives using a ChassisSpeeds
   * @param desiredChassisSpeeds
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds) {
    // Process lock-on
    if (m_lockOnEnabled) {
      var targetedAprilTag = Limelight.getApriltagId();

      // If targetedAprilTag is in validTargets, snap to its offset
      if (targetedAprilTag != -1 && Limelight.tagIdIsASpeakerTarget(targetedAprilTag)) {
        // Calculate the target heading
        var horizontalOffset = Limelight.getHorizontalOffsetFromTarget().getDegrees();
        var robotHeading = getHeading();
        var targetHeading = robotHeading + horizontalOffset;

        // Set the drivetrain to snap to the target heading
        setSnapToSetpoint(targetHeading);

        // If the target is within 5 degrees, set the LEDs to indicate shoot, otherwise
        // quickly pulse red, indicating that the robot is not locked on yet
        if (Math.abs(horizontalOffset) < 5) {
          m_leds.setStripTemporary(LEDSection.solidColor(Color.GREEN));
        } else {
          m_leds.setStripTemporary(LEDSection.pulseColor(Color.RED, 100));
        }
      } else {
        setSnapToGyroEnabled(false);
        m_leds.restoreLastStripState();
      }
    }

    // If we're snapping the angle, calculate and set the rotational speed to reach the setpoint
    if (m_snapToGyroEnabled) {
      d_snapAngle.setDouble(m_snapToRotationController.getSetpoint());
      desiredChassisSpeeds.omegaRadiansPerSecond =
        m_snapToRotationController.calculate(MathUtil.angleModulus(m_gyro.getRotation2d().getRadians()));
    } else {
      d_snapAngle.setDouble(0);
    }

    // Correct drift
    desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_config.Drivetrain.MaxSpeedMetersPerSecond);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Gets the current pose of the drivetrain from odometry
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the position of odometry to the input pose
   * @param pose
   */
  public void setOdometryPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Gets the direction the robot is facing in degrees, CCW+
   * @return
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Stops all drivetrain motors
   */
  public void stopMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }

  /**
   * Gets the module positions as an array in CCW order from FL to FR
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_rearLeftModule.getPosition(),
      m_rearRightModule.getPosition(),
    };
  }

  /**
   * Enabled/disables snap-to gyro control
   */
  public void setSnapToGyroEnabled(boolean enabled) {
    m_snapToGyroEnabled = enabled;
  }

  /**
   * Toggles snap-to gyro control
   */
  public void toggleSnapToGyroEnabled() {
    m_snapToGyroEnabled = !m_snapToGyroEnabled;
  }

  public void setSnapToSetpoint(double angle) {
    var snapAngle = angle;
    if (Robot.onRedAlliance()) {
      snapAngle = snapAngle + 180;
    }

    if (angle >= 360) angle -= 360;

    m_snapToRotationController.setSetpoint(snapAngle);
    setSnapToGyroEnabled(true);
  }

  /**
   * Gets the current chassis speeds of the robot
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
      m_frontLeftModule.getModuleState(),
      m_frontRightModule.getModuleState(),
      m_rearLeftModule.getModuleState(),
      m_rearRightModule.getModuleState()
    );
  }

  /**
   * Takes an input pose and latency and feeds it into the pose estimator
   * @param pose The pose of the robot
   * @param cameraLatencyMs The latency of the camera's pipeline in milliseconds
   */
  public void feedRobotPoseEstimation(Pose2d pose, long cameraLatencyMs) {
    var currentEpochSeconds = Timer.getFPGATimestamp();
    var poseTimestampSeconds = currentEpochSeconds - (cameraLatencyMs / 1000.0);
    m_poseEstimator.addVisionMeasurement(pose, poseTimestampSeconds);
  }

  //#endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    // Update odometry
    var gyroAngle = m_gyro.getRotation2d();
    d_currentHeading.setDouble(gyroAngle.getDegrees());
    m_poseEstimator.update(gyroAngle, getModulePositions());
    m_fieldWidget.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // Update shuffleboard entries
    d_snapToEnabledEntry.setBoolean(m_snapToGyroEnabled);
  }

  //#region Commands

  /**
   * Creates a command that drives the robot using the input controls
   * @param controlSuppliers Controller input
   * @param fieldRelative Whether or not the input is field relative or robot relative
   */
  public Command defaultDriveCommand(SwerveControlSuppliers controlSuppliers, boolean fieldRelative) {
    return this.run(() -> {
        if (Math.abs(controlSuppliers.Rotation.getAsDouble()) > 0.2) {
          setSnapToGyroEnabled(false);
        }

        // Scale speeds cubic
        var forwardY = Controls.cubicScaledDeadband(controlSuppliers.Forward.getAsDouble(), 0.1, 0.3);
        var strafeX = Controls.cubicScaledDeadband(controlSuppliers.Strafe.getAsDouble(), 0.1, 0.3);
        var rotation = Controls.cubicScaledDeadband(controlSuppliers.Rotation.getAsDouble(), 0.1, 0.3);

        // Set speeds to MPS
        strafeX = -controlSuppliers.Strafe.getAsDouble() * m_config.Drivetrain.MaxSpeedMetersPerSecond;
        forwardY = -controlSuppliers.Forward.getAsDouble() * m_config.Drivetrain.MaxSpeedMetersPerSecond;
        rotation = -controlSuppliers.Rotation.getAsDouble() * m_config.Drivetrain.MaxAngularSpeedRadians;

        driveFromCartesianSpeeds(-strafeX, forwardY, rotation, fieldRelative);
      });
  }

  /**
   * Command for resetting the gyro
   */
  public Command resetGyroCommand() {
    return Commands.runOnce(() -> resetGyro());
  }

  /**
   * Enables snap-to control and sets an angle setpoint
   * @param angle
   */
  public Command setSnapToSetpointCommand(double angle) {
    return Commands.runOnce(() -> setSnapToSetpoint(angle));
  }

  /**
   * Disables snap-to control
   */
  public Command disableSnapToCommand() {
    return Commands.runOnce(() -> setSnapToGyroEnabled(false));
  }

  /**
   * Enables lock-on control
   * @return
   */
  public Command enableLockOn() {
    return Commands.runOnce(() -> {
      m_lockOnEnabled = true;
      m_leds.setStripTemporary(LEDSection.pulseColor(Color.RED, 100));
    });
  }

  /**
   * Disables lock-on control
   * @return
   */
  public Command disableLockOn() {
    return Commands.runOnce(() -> {
      m_lockOnEnabled = false;
      m_snapToGyroEnabled = false;
      m_leds.restoreLastStripState();
    });
  }

  /**
   * Feeds the robot's pose from the Limelight into the pose estimator, if a target is detected
   * @return
   */
  public Command estimatePoseCommand() {
    return Commands.runOnce(() -> {
      if (Limelight.getApriltagId() != -1) {
        var robotPose = Limelight.getRobotPose(Alliance.Blue);
        var cameraLatencyMs = Limelight.getTotalLatencyMs();

        feedRobotPoseEstimation(robotPose.toPose2d(), cameraLatencyMs);
      }
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      "Estimate_Pose",
      estimatePoseCommand(),
      "Enable_Lock_On",
      enableLockOn(),
      "Disable_Lock_On",
      disableLockOn()
    );
  }

  //#endregion

  @Override
  public void close() throws Exception {
    DriverStation.reportWarning(">> Drivetrain closing...", false);

    // close all physical resources
    m_gyro.close();
    m_frontLeftModule.close();
    m_frontRightModule.close();
    m_rearLeftModule.close();
    m_rearRightModule.close();
    m_snapToRotationController.close();

    // release memory resources
    m_kinematics = null;
    m_poseEstimator = null;
    m_fieldWidget = null;
  }
}
