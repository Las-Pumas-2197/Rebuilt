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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision camera using the new PhotonVision library API.
 * Drop-in replacement for VisionCamera — swap in Vision.java to use.
 *
 * Key differences from VisionCamera:
 * - Uses 2-arg PhotonPoseEstimator constructor + setPrimaryStrategy()
 * - Uses estimateCoprocMultiTagPose() with explicit single-tag fallback
 * - Enhanced std dev calculation using ambiguity, area, and tag count
 */
public class VisionCameraNew extends SubsystemBase {

  private final String m_name;
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;
  private PhotonCameraSim m_camerasim;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);
  private List<Integer> fiducials = new ArrayList<>();

  public VisionCameraNew(String name, Transform3d intrinsics) {
    m_name = name;
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(k_fieldlayout, intrinsics);
    m_estimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      // 70° horizontal, 56.5° vertical FOV
      // diagonal = 2 * atan(sqrt(tan(35°)^2 + tan(28.25°)^2)) ≈ 87.2°
      double diagFovDeg = Math.toDegrees(2 * Math.atan(Math.sqrt(
          Math.pow(Math.tan(Math.toRadians(35)), 2)
        + Math.pow(Math.tan(Math.toRadians(28.25)), 2))));
      camerasim_properties.setCalibration(960, 738, Rotation2d.fromDegrees(diagFovDeg));
      camerasim_properties.setFPS(10);
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
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
   * Calculates std devs for the estimate using distance, ambiguity, area, and tag count.
   *
   * @param estimate The estimated pose.
   * @param result   The pipeline result.
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

    Matrix<N3, N1> scaled = stddevs.times(scaleFactor);

    String prefix = "Vision/" + m_name + "/";
    SmartDashboard.putNumber(prefix + "NumTags", numTags);
    SmartDashboard.putNumber(prefix + "AvgDist", avgDist);
    SmartDashboard.putNumber(prefix + "AvgAmbiguity", avgAmbiguity);
    SmartDashboard.putNumber(prefix + "AvgArea", avgArea);
    SmartDashboard.putNumber(prefix + "ScaleFactor", scaleFactor);
    SmartDashboard.putNumber(prefix + "StdDevX", scaled.get(0, 0));
    SmartDashboard.putNumber(prefix + "StdDevY", scaled.get(1, 0));
    SmartDashboard.putNumber(prefix + "StdDevTheta", scaled.get(2, 0));

    return scaled;
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

  /** Updates the robot-to-camera transform on the pose estimator. */
  public void updateTransform(Transform3d robotToCamera) {
    m_estimator.setRobotToCameraTransform(robotToCamera);
  }

  public PhotonCameraSim getSimInstance() {
    return m_camerasim;
  }

  @Override
  public void periodic() {
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    results.ifPresentOrElse(result -> {
      // Try multi-tag pose estimation first
      var multiTagEstimate = m_estimator.estimateCoprocMultiTagPose(result);
      if (multiTagEstimate.isPresent()) {
        var est = multiTagEstimate.get();
        estimate = Pair.of(Optional.of(est), calculateStdDevs(est, result));
      } else if (!result.targets.isEmpty()) {
        // Single-tag fallback — uses LOWEST_AMBIGUITY via the configured fallback strategy
        m_estimator.update(result).ifPresent(est -> {
          estimate = Pair.of(Optional.of(est), calculateStdDevs(est, result));
        });
      } else {
        estimate = Pair.of(Optional.empty(), k_ignorestddevs);
      }
    }, () -> estimate = Pair.of(Optional.empty(), k_ignorestddevs));
  }
}
