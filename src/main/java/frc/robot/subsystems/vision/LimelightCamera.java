// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightCamera extends SubsystemBase {

  private final String m_name;
  private final DoubleSupplier m_yawDegreesSupplier;
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);

  /**
   * @param name The NetworkTables name of the Limelight.
   * @param yawDegreesSupplier Supplier for the robot's gyro yaw in degrees for MegaTag2.
   */
  public LimelightCamera(String name, DoubleSupplier yawDegreesSupplier) {
    m_name = name;
    m_yawDegreesSupplier = yawDegreesSupplier;
  }

  /** Calculates standard deviations for the Limelight estimate based on tag metrics. */
  private Matrix<N3, N1> calculateStdDevs(PoseEstimate poseEstimate) {
    if (poseEstimate.tagCount == 0) {
      return k_ignorestddevs;
    }

    Matrix<N3, N1> stddevs = (poseEstimate.tagCount > 1) ? k_multitagstddevs : k_singletagstddevs;

    double distanceFactor = Math.max(1.0, Math.pow(poseEstimate.avgTagDist / 2.0, 2));
    double tagCountFactor = 1.0 / Math.sqrt(poseEstimate.tagCount);
    double clampedArea = Math.max(0.1, Math.min(poseEstimate.avgTagArea, 10.0));
    double areaFactor = 2.0 / clampedArea;

    return stddevs.times(distanceFactor * tagCountFactor * areaFactor);
  }

  /** Returns the current estimate with standard deviations. */
  public Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> getEstimate() {
    return estimate;
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation(m_name, m_yawDegreesSupplier.getAsDouble(), 0, 0, 0, 0, 0);
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      estimate = Pair.of(Optional.empty(), k_ignorestddevs);
      return;
    }

    EstimatedRobotPose robotPose = new EstimatedRobotPose(
        new Pose3d(poseEstimate.pose),
        poseEstimate.timestampSeconds,
        List.of());

    estimate = Pair.of(Optional.of(robotPose), calculateStdDevs(poseEstimate));
  }
}
