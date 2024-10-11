package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swervemodule.ISwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.subsystems.drivetrain.swervemodule.ISwerveModuleIO.SwerveModuleIOOutputs;

public class SwerveModuleSubsystem extends SubsystemBase {

  private ISwerveModuleIO m_swerveio;
  private SwerveModuleIOInputs m_inputs;
  private SwerveModuleIOOutputs m_outputs;

  public SwerveModuleSubsystem(boolean isReal, SwerveModuleMap moduleMap) {
    m_swerveio = isReal ? new SwerveModuleIOReal(moduleMap) : new SwerveModuleIOSim();
    setName(moduleMap.ModuleName);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The state of the module that we'd like to be at in this
   *                     period
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state
    desiredState = optimize(desiredState);

    // Output the optimized desired state
    m_outputs.DesiredState = desiredState;
  }

  /**
   * Optimizes the module angle & drive inversion to ensure the module takes the shortest path to drive at the desired angle
   * @param desiredState
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    Rotation2d currentAngle = m_inputs.ModulePosition.angle;
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))
      );
    } else {
      return desiredState;
    }
  }

  /**
   * Stops both motors within the Module
   */
  public void stopMotors() {
    m_swerveio.stopMotors();
  }

  /**
   * Gets the cumulative SwerveModulePosition of the module
   */
  public SwerveModulePosition getPosition() {
    return m_inputs.ModulePosition;
  }

  /**
   * Gets the instantaneous state of the module
   */
  public SwerveModuleState getModuleState() {
    return m_inputs.ModuleState;
  }

  /**
   * Updates IO
   */
  @Override
  public void periodic() {
    m_inputs = m_swerveio.getInputs();
    m_swerveio.setOutputs(m_outputs);
  }
}
