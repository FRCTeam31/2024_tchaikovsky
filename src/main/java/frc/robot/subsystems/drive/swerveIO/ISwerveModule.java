package frc.robot.subsystems.drive.swerveIO;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {
  public void setDesiredState(SwerveModuleState state);

  public SwerveModuleState getModuleState();

  public SwerveModulePosition getModulePosition();

  public void stopMotors();
}
