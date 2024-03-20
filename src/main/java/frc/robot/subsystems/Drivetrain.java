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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.PulsePattern;
import prime.control.LEDs.Patterns.SolidPattern;
import prime.control.SwerveControlSuppliers;
import prime.movers.IPlannable;
import prime.physics.LimelightPose;

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
   * @param desiredChassisSpeeds
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds) {
    // If we're snapping the angle, calculate and set the rotational speed to reach the setpoint
    if (m_snapToGyroEnabled) {
      desiredChassisSpeeds.omegaRadiansPerSecond =
        m_snapToRotationController.calculate(MathUtil.angleModulus(m_gyro.getRotation2d().getRadians()));

      // If the robot is close to the setpoint, indicate that the robot is aligned
      if (Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < 0.1) {
        m_leds.setStripTemporaryPattern(new SolidPattern(Color.GREEN));
      } else {
        m_leds.setStripTemporaryPattern(new PulsePattern(Color.RED, 5));
      }
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

  /**
   * Sets the snap-to gyro setpoint, converting from degrees to radians
   * @param angle
   */
  public void setSnapToSetpoint(double angle) {
    var snapAngle = angle;
    if (Robot.onRedAlliance()) {
      snapAngle = snapAngle + 180;
    }

    var setpoint = Rotation2d.fromDegrees(snapAngle).getRadians();

    m_snapToRotationController.setSetpoint(MathUtil.angleModulus(setpoint));
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
   * Takes an input pose and latency and feeds it into the pose estimator, if the
   * speed is low enough that the vision measurement is more likely to be accurate
   * @param pose The pose of the robot
   * @param cameraLatencyMs The latency of the camera's pipeline in milliseconds
   */
  public void feedRobotPoseEstimation(LimelightPose llPose) {}

  public boolean withinTrustedVelocity() {
    return (
      getChassisSpeeds().omegaRadiansPerSecond < 0.1 || // 0.1 rad/s is about 6 degrees/s
      getChassisSpeeds().vxMetersPerSecond < 0.2 ||
      getChassisSpeeds().vyMetersPerSecond < 0.2
    );
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

    // If we have a valid target and we're moving in a trusted velocity range, update the pose estimator
    var primaryTarget = Limelight.getApriltagId();
    if (withinTrustedVelocity() && Limelight.isValidApriltag(primaryTarget)) {
      var llPose = Limelight.getRobotPose(Alliance.Blue);

      m_poseEstimator.addVisionMeasurement(llPose.Pose.toPose2d(), llPose.Timestamp, llPose.StdDeviations);
      // Logger.recordOutput("Limelight Pose2d", llPose.Pose.getPose2d());
      // var std = new double[3];
      // for (int i = 0; i < 3; i++) {
      //   std[i] = llPose.StdDeviations.get(i, 0);
      // }
      // Logger.recordOutput("Limelight Standard Deviations", std);
    }
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
      // m_lockOnEnabled = true;
      var targetedAprilTag = Limelight.getApriltagId();

      // If targetedAprilTag is in validTargets, snap to its offset
      if (targetedAprilTag != -1 && Limelight.isSpeakerCenterTarget(targetedAprilTag)) {
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

  /**
   * Disables lock-on control
   * @return
   */
  public Command disableLockOn() {
    return Commands.runOnce(() -> {
      m_lockOnEnabled = false;
      // m_snapToGyroEnabled = false;
      // m_leds.restoreLastStripState();
    });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of("Enable_Lock_On", enableLockOn(), "Disable_Lock_On", disableLockOn());
  }

  //#endregion

  @Override
  public void close() throws Exception {
    DriverStation.reportWarning(">> [DRIVE] Closing...", false);

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
