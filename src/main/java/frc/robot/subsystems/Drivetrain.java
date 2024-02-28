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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.config.RobotConfig;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.movers.IPlannable;

public class Drivetrain extends SubsystemBase implements IPlannable {

  // Container for robot configuration
  private RobotConfig m_config;

  // Shuffleboard configuration
  private final StructArrayPublisher<SwerveModuleState> m_measuredSwerveStatesPublisher;
  private final StructArrayPublisher<SwerveModuleState> m_desiredSwerveStatesPublisher;
  private final DoublePublisher m_gyroPublisher;

  private ShuffleboardTab d_driveTab = Shuffleboard.getTab("Drivetrain");
  public GenericEntry d_snapToEnabledEntry = d_driveTab
    .add("SnapTo Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
  public GenericEntry d_inHighGearEntry = d_driveTab
    .add("In High Gear", false)
    .getEntry();
  private GenericEntry d_gyroAngle = d_driveTab
    .add("Gyro Angle", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withProperties(Map.of("major tick spacing", 15, "starting angle", 0))
    .getEntry();

  // Gyro and Kinematics
  public Pigeon2 m_gyro;
  public SwerveDriveKinematics m_kinematics;
  public boolean m_inHighGear = true;

  // Swerve Modules, in CCW order from FL to FR
  // SwerveModule m_frontLeftModule, m_rearLeftModule, m_rearRightModule, m_frontRightModule;
  SwerveModule m_frontLeftModule, m_frontRightModule, m_rearLeftModule, m_rearRightModule;

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

    // Create swerve modules, kinematics, and odometry
    m_measuredSwerveStatesPublisher =
      NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
        .publish();

    m_desiredSwerveStatesPublisher =
      NetworkTableInstance
        .getDefault()
        .getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState.struct)
        .publish();

    m_gyroPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("/Gyro").publish();

    m_kinematics =
      new SwerveDriveKinematics(
        m_config.FrontLeftSwerveModule.getModuleLocation(),
        m_config.FrontRightSwerveModule.getModuleLocation(),
        m_config.RearLeftSwerveModule.getModuleLocation(),
        m_config.RearRightSwerveModule.getModuleLocation()
      );
    createSwerveModulesAndOdometry();
    m_inHighGear = config.Drivetrain.StartInHighGear;

    // Configure field
    m_field = new Field2d();
    d_driveTab.add("Field", m_field).withWidget(BuiltInWidgets.kField);

    // Configure snap-to PID

    m_snapToRotationController =
      new PIDController(
        m_config.Drivetrain.SnapToPID.kP,
        m_config.Drivetrain.SnapToPID.kI,
        m_config.Drivetrain.SnapToPID.kD,
        0.02
      );

    // m_snapToRotationController.setTolerance(
    //   Math.toRadians(30),
    //   Math.toRadians(30)
    // );

    m_snapToRotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_snapToRotationController.setSetpoint(0);
    d_driveTab
      .add("SnapTo PID", m_snapToRotationController)
      .withWidget(BuiltInWidgets.kPIDController)
      .withPosition(0, 0);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      m_config.Drivetrain.getHolonomicPathFollowerConfig(),
      () -> false, // Method to determine whether or not to flip the path
      this // Reference to this subsystem to set requirements
    );
    // Robot.m_autoChooser = AutoBuilder.buildAutoChooser("1m Auto");

  }

  // #region Methods
  // Creates the swerve modules and starts odometry
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
        getModulePositions(),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      );

    // in CCW order from FL to FR
    m_swerveModules =
      new SwerveModule[] {
        m_frontLeftModule,
        m_frontLeftModule,
        m_rearLeftModule,
        m_rearRightModule,
      };
  }

  // Resets the Gyro
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
        -m_snapToRotationController.calculate(
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
    // var xSpeed = desiredChassisSpeeds.vxMetersPerSecond;
    // var ySpeed = desiredChassisSpeeds.vyMetersPerSecond;

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
      desiredChassisSpeeds
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,
      m_config.Drivetrain.MaxSpeedMetersPerSecond
    );

    m_desiredSwerveStatesPublisher.set(swerveModuleStates);

    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  // Gets the current pose of the drivetrain from odometry
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Resets the position of odometry to the current position, minus 90
  public void resetOdometry(Pose2d pose) {
    m_swerveModulePositions[0] = m_frontLeftModule.getPosition();
    m_swerveModulePositions[1] = m_frontRightModule.getPosition();
    m_swerveModulePositions[2] = m_rearLeftModule.getPosition();
    m_swerveModulePositions[3] = m_rearRightModule.getPosition();

    m_odometry.resetPosition(
      m_gyro.getRotation2d(),
      m_swerveModulePositions,
      pose
    );
  }

  // Gets the direction the robot is facing in degrees, CCW+
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  // Sets the virtual gearbox shifter
  public void setShift(boolean inHighGear) {
    m_inHighGear = inHighGear;
  }

  // Toggles the virtual gearbox shifter
  public void toggleShifter() {
    m_inHighGear = !m_inHighGear;
  }

  // Stops all drivetrain motors
  public void stopMotors() {
    m_frontLeftModule.stopMotors();
    m_frontRightModule.stopMotors();
    m_rearLeftModule.stopMotors();
    m_rearRightModule.stopMotors();
  }

  // Sets the modules all to a single heading
  public void setWheelAngles(Rotation2d angle) {
    m_frontLeftModule.setDesiredAngle(angle);
    m_frontRightModule.setDesiredAngle(angle);
    m_rearLeftModule.setDesiredAngle(angle);
    m_rearRightModule.setDesiredAngle(angle);
  }

  /**
   * Sets the drive voltage of all modules. Used for SysID routines
   *
   * @param voltage
   */
  public void setModuleDriveVoltages(Measure<Voltage> voltage) {
    // Lock the wheels facing forward
    setWheelAngles(Rotation2d.fromDegrees(0));

    m_frontLeftModule.setDriveVoltage(voltage.magnitude());
    m_frontRightModule.setDriveVoltage(voltage.magnitude());
    m_rearLeftModule.setDriveVoltage(voltage.magnitude());
    m_rearRightModule.setDriveVoltage(voltage.magnitude());
  }

  /**
   * Gets the module positions as an array
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
  public void setSnapToGyroControl(boolean enabled) {
    m_snapToGyroEnabled = enabled;
  }

  /**
   * Toggles snap-to gyro control
   */
  public void toggleSnapToGyroControl() {
    m_snapToGyroEnabled = !m_snapToGyroEnabled;
    m_snapToRotationController.close();
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

  // Updates odometry and any other periodic drivetrain events
  @Override
  public void periodic() {
    // Update odometry
    var gyroAngle = m_gyro.getRotation2d();
    d_gyroAngle.setDouble(gyroAngle.getDegrees());
    m_gyroPublisher.set(gyroAngle.getRadians());

    m_measuredSwerveStatesPublisher.set(
      new SwerveModuleState[] {
        m_frontLeftModule.getModuleState(),
        m_frontRightModule.getModuleState(),
        m_rearLeftModule.getModuleState(),
        m_rearRightModule.getModuleState(),
      }
    );

    // m_desiredSwerveStatesPublisher.set(SwerveModuleState[]{
    //     m_frontLeftModule.getModuleState(),
    //     m_frontRightModule.getModuleState(),
    //     m_rearLeftModule.getModuleState(),
    //     m_rearRightModule.getModuleState(),

    // });

    m_desiredSwerveStatesPublisher.set(
      new SwerveModuleState[] {
        m_frontLeftModule.getModuleState(),
        m_frontRightModule.getModuleState(),
        m_rearLeftModule.getModuleState(),
        m_rearRightModule.getModuleState(),
      }
    );
    var robotPose = m_odometry.update(gyroAngle, getModulePositions());
    m_field.setRobotPose(robotPose);

    // Update shuffleboard entries
    d_snapToEnabledEntry.setBoolean(m_snapToGyroEnabled);
    d_inHighGearEntry.setBoolean(m_inHighGear);
  }

  // #endregion

  //#region Commands
  // Creates a command that drives the robot using the default controls
  public Command defaultDriveCommand(
    DoubleSupplier ySupplier,
    DoubleSupplier xSupplier,
    DoubleSupplier rotationSupplier,
    boolean fieldRelative
  ) {
    return this.run(() -> {
        var strafeX =
          -xSupplier.getAsDouble() *
          m_config.Drivetrain.MaxSpeedMetersPerSecond;
        var forwardY =
          -ySupplier.getAsDouble() *
          m_config.Drivetrain.MaxSpeedMetersPerSecond;
        var rotation =
          -rotationSupplier.getAsDouble() *
          m_config.Drivetrain.MaxAngularSpeedRadians;

        driveFromCartesianSpeeds(-strafeX, forwardY, rotation, fieldRelative);
      });
  }

  // Command for resetting the gyro
  public Command resetGyroCommand() {
    return this.runOnce(() -> resetGyro());
  }

  // Command for resetting the Robots odometry
  public Command resetOdometryCommand(Pose2d pose) {
    return this.runOnce(() -> resetOdometry(pose));
  }

  // Command for toggling the shifter
  public Command toggleShifterCommand() {
    return this.runOnce(() -> toggleShifter());
  }

  // Command for setting the angle of the wheels
  public Command setWheelAnglesCommand(Rotation2d angle) {
    return this.runOnce(() -> setWheelAngles(angle));
  }

  /**
   * Creates a command that enables snap to gyro control
   */
  public Command setSnapToGyroControlCommand(boolean enabled) {
    return this.runOnce(() -> setSnapToGyroControl(enabled));
  }

  // Command for toggling Snap-To controls
  public Command toggleSnapToAngleCommand() {
    return this.runOnce(() -> {
        toggleSnapToGyroControl();
      });
  }

  // Command for driving with Snap-To controls enabled
  public Command driveWithSnapToAngleCommand(double angle) {
    return this.runOnce(() -> {
        setSnapToGyroControl(true);
        m_snapToRotationController.setSetpoint(angle);
      });
  }

  public Map<String, Command> getNamedCommands() {
    return Map.of(
      // "Example_Command", exampleCommand(),
    );
  }

  //#endregion

  //#region SysID Routines

  /**
   * Factory method for creating a quasistatic SysID routine for the drivetrain
   * @param direction
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return generateSysIdRoutine().quasistatic(direction);
  }

  /**
   * Factory method for creating a dynamic SysID routine for the drivetrain
   * @param direction
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return generateSysIdRoutine().dynamic(direction);
  }

  /**
   * Generates a SysID routine for the drivetrain
   * @return
   */
  public SysIdRoutine generateSysIdRoutine() {
    var config = new SysIdRoutine.Config(
      Units.Volts.per(Units.Second).of(1),
      Units.Volts.of(5),
      Units.Seconds.of(15),
      state -> {
        SmartDashboard.putString(
          "Drive/SysID Current Routine",
          state.toString()
        );
      }
    );

    var mechanism = new Mechanism(
      this::setModuleDriveVoltages,
      this::logMotors,
      this,
      getName()
    );

    return new SysIdRoutine(config, mechanism);
  }

  /**
   * Logs the voltage, distance, and velocity of each swerve module to SysID
   * @param log
   */
  private void logMotors(SysIdRoutineLog log) {
    log
      .motor(m_frontLeftModule.getName())
      .voltage(Units.Volts.of(m_frontLeftModule.getDriveVoltage()))
      .linearPosition(
        Units.Meters.of(m_frontLeftModule.getPosition().distanceMeters)
      )
      .linearVelocity(
        Units.MetersPerSecond.of(
          m_frontLeftModule.getModuleState().speedMetersPerSecond
        )
      );

    log
      .motor(m_frontRightModule.getName())
      .voltage(Units.Volts.of(m_frontRightModule.getDriveVoltage()))
      .linearPosition(
        Units.Meters.of(m_frontRightModule.getPosition().distanceMeters)
      )
      .linearVelocity(
        Units.MetersPerSecond.of(
          m_frontRightModule.getModuleState().speedMetersPerSecond
        )
      );

    log
      .motor(m_rearLeftModule.getName())
      .voltage(Units.Volts.of(m_rearLeftModule.getDriveVoltage()))
      .linearPosition(
        Units.Meters.of(m_rearLeftModule.getPosition().distanceMeters)
      )
      .linearVelocity(
        Units.MetersPerSecond.of(
          m_rearLeftModule.getModuleState().speedMetersPerSecond
        )
      );

    log
      .motor(m_rearRightModule.getName())
      .voltage(Units.Volts.of(m_rearRightModule.getDriveVoltage()))
      .linearPosition(
        Units.Meters.of(m_rearRightModule.getPosition().distanceMeters)
      )
      .linearVelocity(
        Units.MetersPerSecond.of(
          m_rearRightModule.getModuleState().speedMetersPerSecond
        )
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
    m_odometry = null;
    m_field = null;
  }
}
