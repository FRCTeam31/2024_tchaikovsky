package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem.DriveMap;
import frc.robot.subsystems.drivetrain.swervemodule.ModuleController;

public class DrivetrainIOReal implements IDrivetrainIO {

  private DrivetrainIOInputs m_inputs;

  private Pigeon2 m_gyro;
  private ModuleController m_swerveController;
  private PIDController m_snapAngleController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  public DrivetrainIOReal() {
    m_inputs = new DrivetrainIOInputs();

    // Create gyro
    m_gyro = new Pigeon2(DriveMap.PigeonId);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());

    // Create swerve modules
    m_swerveController = new ModuleController(true);

    // Configure snap-to PID
    m_snapAngleController = DriveMap.SnapToPID.createPIDController(0.02);
    m_snapAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Create kinematics in order FL, FR, RL, RR
    m_kinematics =
      new SwerveDriveKinematics(
        DriveMap.FrontLeftSwerveModule.getModuleLocation(),
        DriveMap.FrontRightSwerveModule.getModuleLocation(),
        DriveMap.RearLeftSwerveModule.getModuleLocation(),
        DriveMap.RearRightSwerveModule.getModuleLocation()
      );

    // Create pose estimator
    m_poseEstimator =
      new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        m_swerveController.getPositions(),
        new Pose2d()
      );
  }

  @Override
  public DrivetrainIOInputs getInputs() {
    m_inputs.GyroAngle = m_gyro.getRotation2d();
    m_inputs.GyroAccelX = m_gyro.getAccelerationX().getValueAsDouble();
    m_inputs.GyroAccelY = m_gyro.getAccelerationY().getValueAsDouble();
    m_inputs.GyroAccelZ = m_gyro.getAccelerationZ().getValueAsDouble();
    m_inputs.RobotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(m_swerveController.getModuleStates());
    var modulePositions = m_swerveController.getPositions();
    m_inputs.EstimatedRobotPose = m_poseEstimator.update(m_inputs.GyroAngle, modulePositions);
    m_inputs.ModulePositions = modulePositions;

    return m_inputs;
  }

  @Override
  public void setOutputs(DrivetrainIOOutputs outputs) {
    if (outputs.SnapEnabled) {
      m_snapAngleController.setSetpoint(outputs.SnapSetpoint.getRadians());
    }

    switch (outputs.ControlMode) {
      case kRobotRelative:
        driveRobotRelative(outputs.DesiredChassisSpeeds, outputs.SnapEnabled);
        break;
      case kFieldRelative:
        drivePathPlanner(outputs.DesiredChassisSpeeds);
        break;
      default:
        break;
    }
  }

  public void resetGyro() {
    m_gyro.setYaw(Robot.onBlueAlliance() ? 180 : 0);

    m_poseEstimator.resetPosition(
      m_gyro.getRotation2d(),
      m_swerveController.getPositions(),
      m_poseEstimator.getEstimatedPosition()
    );
  }

  public void setEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_swerveController.getPositions(), pose);
  }

  public void addPoseEstimatorVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviations);
  }

  /**
   * Drives robot-relative using a ChassisSpeeds
   * @param desiredChassisSpeeds The desired speeds of the robot
   */
  private void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean snapAngleEnabled) {
    // If snap-to is enabled, calculate and override the input rotational speed to reach the setpoint
    if (snapAngleEnabled) {
      var currentRotationRadians = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());
      desiredChassisSpeeds.omegaRadiansPerSecond = m_snapAngleController.calculate(currentRotationRadians);

      // Report back if snap is on-target
      m_inputs.SnapOnTarget = Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < 0.1;
    }

    // Correct drift by taking the input speeds and converting them to a desired per-period speed. This is known as "discretizing"
    desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

    // Calculate the module states from the chassis speeds
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveMap.MaxSpeedMetersPerSecond);

    // Set the desired states for each module
    m_swerveController.setDesiredStates(swerveModuleStates);
  }

  /**
   * Facilitates driving using PathPlanner generated speeds
   * @param robotRelativeSpeeds
   */
  private void drivePathPlanner(ChassisSpeeds robotRelativeSpeeds) {
    if (Robot.onRedAlliance()) {
      // If we're on the red alliance, we need to flip the gyro
      var gyroAngle = m_gyro.getRotation2d().plus(Rotation2d.fromDegrees(180));

      // Convert the robot-relative speeds to field-relative speeds with the flipped gyro
      var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, gyroAngle);

      // Convert back to robot-relative speeds, also with the flipped gyro
      driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, gyroAngle), false);
    } else {
      driveRobotRelative(robotRelativeSpeeds, false);
    }
  }
}
