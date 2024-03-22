// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.limelightIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

/**
 * climber subsystem hardware interface.
 */
public interface ILimelight {
  /**
   * Contains all of the input data received from hardware.
   */
  @AutoLog
  public static class LimelightInputs {

    public int AprilTagId = -1;
    public double[] BotPoseBlue = new double[11];
    public Rotation2d TargetHorizontalOffset = new Rotation2d();
  }

  /**
   * Updates the set of loggable inputs.
   */
  public default void updateInputs(LimelightInputs inputs) {}

  public static boolean isSpeakerCenterTarget(int apriltagId) {
    return apriltagId == 4 || apriltagId == 7;
  }

  public static boolean isValidApriltag(int apriltagId) {
    return apriltagId >= 1 && apriltagId <= 16;
  }

  /**
   * Calculates a trust value based on the number of tags in view.
   * @return
   */
  public static Matrix<N3, N1> calculateTrust(double tagCount) {
    return VecBuilder.fill(tagCount >= 2.0 ? 0.5 : 10, tagCount >= 2.0 ? 0.5 : 10, 999999);
  }
}
