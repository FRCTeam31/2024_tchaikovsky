package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.config.RobotConfig;
import java.util.Map;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.PulsePattern;
import prime.control.LEDs.Patterns.SolidPattern;
import prime.control.SwerveControlSuppliers;
import prime.physics.LimelightPose;

public class Drivetrain extends SubsystemBase {

  // Container for robot configuration
  private RobotConfig m_config;

  // Shuffleboard configuration
  private ShuffleboardTab d_drivetrainTab = Shuffleboard.getTab("Drivetrain");
  public Field2d m_fieldWidget;
  public GenericEntry d_snapToEnabledEntry = d_drivetrainTab
    .add("SnapTo Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0, 0)
    .withSize(2, 2)
    .getEntry();
  private GenericEntry d_currentHeading = d_drivetrainTab
    .add("Current Heading", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(14, 0)
    .withSize(4, 5)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();
  private GenericEntry d_snapAngle = d_drivetrainTab
    .add("SnapTo Angle", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(2, 0)
    .withSize(4, 5)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();

  // Driver Tab widgets
  private GenericEntry d_tidEntry = RobotContainer.DriverTab
    .add("Targeted APTag", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(11, 3)
    .withSize(2, 1)
    .getEntry();
  private GenericEntry d_txEntry = RobotContainer.DriverTab
    .add("Target X Offset", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Min", -29.8, "Max", 29.8))
    .withPosition(11, 4)
    .withSize(2, 3)
    .getEntry();

  // Gyro and swerve modules in CCW order from FL to FR
  public Pigeon2 m_gyro;
  private SwerveController m_swerveController;

  // Kinematics, odometry, and field widget
  public Limelight Limelight;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  // Snap to Gyro Angle PID & Lock-on
  public boolean m_lockOnEnabled = false;
  public boolean m_snapToGyroEnabled = false;
  public PIDController m_snapToRotationController;

  private PwmLEDs m_leds;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(RobotConfig config, PwmLEDs leds) {
    setName("Drivetrain");
    m_config = config;

    m_leds = leds;

    // Create gyro
    m_gyro = new Pigeon2(config.Drivetrain.PigeonId);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    RobotContainer.DriverTab
      .add("Current Heading", m_gyro)
      .withWidget(BuiltInWidgets.kGyro)
      .withPosition(8, 3)
      .withSize(3, 3)
      .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0));

    // Create swerve modules in CCW order from FL to FR
    m_swerveController = new SwerveController(config, config.Drivetrain.DrivePID, config.Drivetrain.SteeringPID);

    // Create kinematics and odometry tooling
    Limelight = new Limelight();
    RobotContainer.DriverTab
      .addCamera("Limelight Stream", "LL2", "http://limelight.local:5800/stream.mjpg")
      .withSize(8, 4)
      .withPosition(0, 0)
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("Show controls", false, "Show crosshair", false));
    m_kinematics =
      new SwerveDriveKinematics(
        m_config.FrontLeftSwerveModule.getModuleLocation(),
        m_config.FrontRightSwerveModule.getModuleLocation(),
        m_config.RearLeftSwerveModule.getModuleLocation(),
        m_config.RearRightSwerveModule.getModuleLocation()
      );
    m_poseEstimator =
      new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), getModulePositions(), new Pose2d());

    // Create field widget, add it to the dashboard tabs, and feed it PathPlanner's current path poses
    m_fieldWidget = new Field2d();
    d_drivetrainTab.add("Field", m_fieldWidget).withWidget(BuiltInWidgets.kField).withPosition(6, 0).withSize(8, 5);
    RobotContainer.DriverTab
      .add("Field", m_fieldWidget)
      .withWidget(BuiltInWidgets.kField)
      .withPosition(8, 0)
      .withSize(5, 3);
    PathPlannerLogging.setLogActivePathCallback(poses -> m_fieldWidget.getObject("path").setPoses(poses));

    // Configure snap-to PID
    m_snapToRotationController = m_config.Drivetrain.SnapToPID.getPIDController(0.02);
    m_snapToRotationController.enableContinuousInput(-Math.PI, Math.PI);
    d_drivetrainTab
      .add("SnapTo PID", m_snapToRotationController)
      .withWidget(BuiltInWidgets.kPIDController)
      .withPosition(0, 2);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setEstimatorPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig(
        m_config.Drivetrain.PathingTranslationPid.toPIDConstants(),
        m_config.Drivetrain.PathingRotationPid.toPIDConstants(),
        m_config.Drivetrain.MaxSpeedMetersPerSecond,
        m_config.Drivetrain.MaxAccelerationMetersPerSecondSquared,
        new ReplanningConfig(true, true)
      ),
      Robot::onRedAlliance, // BooleanSupplier to tell PathPlanner whether or not to flip the path over the Y midline of the field
      this // Reference to this subsystem to set requirements
    );
  }

  //#region Control methods

  // Resets the Gyro
  public void resetGyro() {
    m_gyro.setYaw(Robot.onRedAlliance() ? 180 : 0);
  }

  /**
   * Drives the robot using cartesian speeds, scaled to the max speed of the robot in meters per second
   * @param inputYMPS The sideways motion of the chassis in meters per second
   * @param inputXMPS The forward motion of the chassis in meters per second
   * @param inputRotationRadiansPS The rotational motion of the chassis in radians per second
   */
  public void driveFieldRelative(double inputXMPS, double inputYMPS, double inputRotationRadiansPS) {
    // Build chassis speeds
    ChassisSpeeds desiredChassisSpeeds;
    var invert = Robot.onRedAlliance() ? -1 : 1;

    // Drive the robot with the driver-relative inputs, converting them to field-relative
    desiredChassisSpeeds =
      ChassisSpeeds.fromFieldRelativeSpeeds(
        inputYMPS * invert, // Use Y as X for field-relative
        inputXMPS * invert, // Use X as Y for field-relative
        inputRotationRadiansPS,
        m_gyro.getRotation2d()
      );

    drive(desiredChassisSpeeds);
  }

  /**
   * Drives using a ChassisSpeeds
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds) {
    // If snap-to is enabled, calculate and set the rotational speed to reach the setpoint
    if (m_snapToGyroEnabled) {
      var currentRotationRadians = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());
      desiredChassisSpeeds.omegaRadiansPerSecond = m_snapToRotationController.calculate(currentRotationRadians);

      // Use the LEDs to indicate how close the robot is to being aligned
      if (Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < 0.1) {
        m_leds.setStripTemporaryPattern(new SolidPattern(Color.GREEN));
      } else {
        m_leds.setStripTemporaryPattern(new PulsePattern(Color.RED, 0.5));
      }
    }

    // Correct drift by taking the input speeds and converting them to a desired per-period speed. This is known as "discretizing"
    desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_config.Drivetrain.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    m_swerveController.setDesiredStates(swerveModuleStates);
  }

  /**
   * Gets the current pose of the drivetrain from odometry
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the position of odometry to the input pose
   * @param pose The pose to reset the estimator to
   */
  public void setEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Gets the direction the robot is facing in degrees, CCW+
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Stops all drivetrain motors
   */
  public void stopMotors() {
    m_swerveController.stopAllMotors();
  }

  /**
   * Gets the module positions as an array in order FL, FR, RL, RR
   */
  public SwerveModulePosition[] getModulePositions() {
    return m_swerveController.getPositions();
  }

  /**
   * Enabled/disables snap-to control
   */
  public void setSnapToGyroEnabled(boolean enabled) {
    m_snapToGyroEnabled = enabled;
    if (!enabled) m_leds.restorePersistentStripState();
  }

  /**
   * Toggles snap-to control on/off
   */
  public void toggleSnapToGyroEnabled() {
    m_snapToGyroEnabled = !m_snapToGyroEnabled;
    if (!m_snapToGyroEnabled) m_leds.restorePersistentStripState();
  }

  /**
   * Sets the snap-to gyro setpoint, converting from degrees to radians
   * @param angle The angle to snap to in degrees
   */
  public void setSnapToSetpoint(double angle) {
    var setpoint = MathUtil.angleModulus(Rotation2d.fromDegrees(angle).getRadians());

    m_snapToRotationController.setSetpoint(setpoint);
    setSnapToGyroEnabled(true);
  }

  /**
   * Gets the current chassis speeds of the robot
   */
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(m_swerveController.getModuleStates());
  }

  /**
   * Checks if the robot is moving at a velocity slow enough to trust the vision measurements
   */
  public boolean withinTrustedVelocity() {
    var speeds = getRobotRelativeChassisSpeeds();

    return (
      speeds.omegaRadiansPerSecond < 0.1 || // 0.1 rad/s is about 6 degrees/s
      speeds.vxMetersPerSecond < 0.2 ||
      speeds.vyMetersPerSecond < 0.2
    );
  }

  /**
   * Checks if the vision estimation is trustworthy using the disance and area of the target
   * @param llPose The pose from the limelight
   */
  public boolean isTrustedEstimation(LimelightPose llPose) {
    return llPose.AvgTagDistanceMeters < 4;
    // && llPose.AvgTagArea < 0.50;
  }

  //#endregion

  /**
   * Updates odometry and any other periodic drivetrain events
   */
  @Override
  public void periodic() {
    var gyroAngle = m_gyro.getRotation2d();
    d_currentHeading.setDouble(gyroAngle.getDegrees());
    SmartDashboard.putNumber("Drive/Gyro (deg)", gyroAngle.getDegrees());

    // Level2 Logging
    var chassisSpeed = getRobotRelativeChassisSpeeds();
    SmartDashboard.putNumber("Drive/Measured/Robot-X (m/s)", chassisSpeed.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/Measured/Robot-Y (m/s)", chassisSpeed.vyMetersPerSecond);
    SmartDashboard.putNumber("Drive/Measured/Robot-Z (rad/s)", chassisSpeed.omegaRadiansPerSecond);

    var estimatedPose = m_poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Drive/Estimated X (m)", estimatedPose.getX());
    SmartDashboard.putNumber("Drive/Estimator Y (m)", estimatedPose.getY());
    SmartDashboard.putNumber("Drive/Estimator Rotation (deg)", estimatedPose.getRotation().getDegrees());

    /// Pose estimation
    // If we have a valid target and we're moving in a trusted velocity range, update the pose estimator
    var primaryTarget = Limelight.getApriltagId();
    var isValidTarget = Limelight.isValidApriltag(primaryTarget);
    var withinTrustedVelocity = withinTrustedVelocity();
    d_tidEntry.setDouble(primaryTarget);
    d_txEntry.setDouble(Limelight.getHorizontalOffsetFromTarget().getDegrees());
    SmartDashboard.putBoolean("Drive/PoseEstimation/IsValidTarget", isValidTarget);
    SmartDashboard.putBoolean("Drive/PoseEstimation/WithinTrustedVelocity", withinTrustedVelocity);
    if (isValidTarget && withinTrustedVelocity) {
      var llPose = Limelight.getRobotPose(Alliance.Blue);

      // Check that the estimation target is not too far away or too large
      var isTrustedEstimation = isTrustedEstimation(llPose);
      SmartDashboard.putBoolean("Drive/PoseEstimation/IsTrustedEstimation", isTrustedEstimation);
      if (isTrustedEstimation) {
        m_poseEstimator.addVisionMeasurement(llPose.Pose.toPose2d(), llPose.Timestamp, llPose.StdDeviations);
      }
    }

    // Update odometry
    m_poseEstimator.update(gyroAngle, getModulePositions());
    m_fieldWidget.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // Update shuffleboard entries
    d_snapToEnabledEntry.setBoolean(m_snapToGyroEnabled);
    d_snapAngle.setDouble(m_snapToRotationController.getSetpoint());
  }

  //#region Commands

  /**
   * Creates a command that drives the robot using input controls
   * @param controlSuppliers Controller input suppliers
   */
  public Command defaultDriveCommand(SwerveControlSuppliers controlSuppliers) {
    return this.run(() -> {
        // If the driver is trying to rotate the robot, disable snap-to control
        if (Math.abs(controlSuppliers.Z.getAsDouble()) > 0.2) {
          setSnapToGyroEnabled(false);
          m_leds.restorePersistentStripState();
        }

        // Convert inputs to MPS
        var inputXMPS = controlSuppliers.X.getAsDouble() * m_config.Drivetrain.MaxSpeedMetersPerSecond;
        var inputYMPS = controlSuppliers.Y.getAsDouble() * m_config.Drivetrain.MaxSpeedMetersPerSecond;
        var inputRotationRadiansPS = controlSuppliers.Z.getAsDouble() * m_config.Drivetrain.MaxAngularSpeedRadians;

        driveFieldRelative(-inputXMPS, inputYMPS, -inputRotationRadiansPS);
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
    return Commands.run(() -> {
      var targetedAprilTag = Limelight.getApriltagId();

      // If targetedAprilTag is in validTargets, snap to its offset
      if (Limelight.isSpeakerCenterTarget(targetedAprilTag)) {
        // Calculate the target heading
        var horizontalOffsetDeg = Limelight.getHorizontalOffsetFromTarget().getDegrees();
        var robotHeadingDeg = getHeading();
        var targetHeadingDeg = robotHeadingDeg - horizontalOffsetDeg;

        // Set the drivetrain to snap to the target heading
        setSnapToSetpoint(targetHeadingDeg);
      } else {
        setSnapToGyroEnabled(false);
      }
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of("Enable_Lock_On", enableLockOn(), "Disable_Snap_To", disableSnapToCommand());
  }
  //#endregion
}
