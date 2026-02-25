// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;
import static frc.robot.utils.Constants.VisionConstants.k_ignorestddevs;
import static frc.robot.utils.Constants.VisionConstants.k_multitagstddevs;
import static frc.robot.utils.Constants.VisionConstants.k_singletagstddevs;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);
  private List<Integer> fiducials = new ArrayList<>();

  public VisionCamera(String name, Transform3d intrinsics) {
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(
        k_fieldlayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        intrinsics);
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Returns a list of the current visible fiducials.
   *
   * @return List of integers containing the visible targets in the results.
   */
  public List<Integer> getFiducialIDs() {
    fiducials.clear();
    results.ifPresent(res -> {
      for (var target : res.targets) {
        fiducials.add(target.fiducialId);
      }
    });
    return fiducials;
  }

  /**
   * Use to calculate the std devs for the resultant estimate. Used internally only.
   * @param estimate The estimate.
   * @param result The results.
   * @return The calculated standard deviations.
   */
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, PhotonPipelineResult result) {
    int numTags = result.targets.size();

    if (numTags == 0) {
      return k_ignorestddevs;
    }

    double avgDist = 0.0;
    double avgAmbiguity = 0.0;
    double totalArea = 0.0;
    int validTags = 0;

    for (var target : result.targets) {
      Optional<Pose3d> tagPose = m_estimator.getFieldTags().getTagPose(target.fiducialId);
      if (tagPose.isEmpty()) continue;

      avgDist += tagPose.get().toPose2d().getTranslation()
          .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
      avgAmbiguity += target.getPoseAmbiguity();
      totalArea += target.getArea();
      validTags++;
    }

    if (validTags == 0) {
      return k_ignorestddevs;
    }

    avgDist /= validTags;
    avgAmbiguity /= validTags;
    double avgArea = totalArea / validTags;

    // Reject high-ambiguity single-tag estimates entirely
    if (numTags == 1 && avgAmbiguity > 0.2) {
      return k_ignorestddevs;
    }

    // Reject any estimate with very high ambiguity
    if (avgAmbiguity > 0.5) {
      return k_ignorestddevs;
    }

    Matrix<N3, N1> stddevs = (numTags > 1) ? k_multitagstddevs : k_singletagstddevs;

    // Distance factor: exponential growth beyond 2m baseline
    double distanceFactor = Math.max(1.0, Math.pow(avgDist / 2.0, 2));

    // Tag count factor: more tags = more confidence (sqrt scaling)
    double tagCountFactor = 1.0 / Math.sqrt(numTags);

    // Ambiguity factor: penalize uncertain estimates
    double ambiguityFactor = 1.0 + (avgAmbiguity * 10.0);

    // Area factor: larger tags in frame = more confidence
    double clampedArea = Math.max(0.1, Math.min(avgArea, 10.0));
    double areaFactor = 2.0 / clampedArea;

    double scaleFactor = distanceFactor * tagCountFactor * ambiguityFactor * areaFactor;

    return stddevs.times(scaleFactor);
  }

  /** Returns the most recent pipeline result. */
  public Optional<PhotonPipelineResult> getResults() {
    return results;
  }

  /** Returns the current estimate with standard deviations. */
  public Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> getEstimate() {
    return estimate;
  }

  /** Returns a boolean with the connection status of the camera. */
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  @Override
  public void periodic() {
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    results.ifPresentOrElse(result -> {
      m_estimator.update(result).ifPresent(est -> {
        estimate = Pair.of(Optional.of(est), calculateStdDevs(est, result));
      });
    }, () -> estimate = Pair.of(Optional.empty(), k_ignorestddevs));
  }
}
