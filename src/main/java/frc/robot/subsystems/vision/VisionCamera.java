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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_estimator;
  private final String m_name;
  private final Transform3d m_robotToCamera;
  private PhotonCameraSim m_camerasim;

  private Optional<PhotonPipelineResult> results = Optional.empty();
  private Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimate = Pair.of(Optional.empty(), k_ignorestddevs);
  private List<Integer> fiducials = new ArrayList<>();

  public VisionCamera(String name, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(k_fieldlayout, robotToCamera);
    m_name = name;
    m_robotToCamera = robotToCamera;

    if (RobotBase.isSimulation()) {
      SimCameraProperties camerasim_properties = new SimCameraProperties();
      camerasim_properties.setFPS(10);
      m_camerasim = new PhotonCameraSim(m_camera, camerasim_properties);
      m_camerasim.enableRawStream(true);
      m_camerasim.enableDrawWireframe(true);
    }
  }

  // returns a list of the current visible fiducials.
  public List<Integer> getFiducialIDs() {
    fiducials.clear();
    results.ifPresent(res -> {
      for (var target : res.targets) {
        fiducials.add(target.fiducialId);
      }
    });
    return fiducials;
  }

  // calculates std devs for the estimate
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate, PhotonPipelineResult result) {
    int numTags = result.targets.size();

    if (numTags == 0) {
      return k_ignorestddevs;
    }

    double avgDist = 0.0;
    int validTags = 0;

    for (var target : result.targets) {
      Optional<Pose3d> tagPose = m_estimator.getFieldTags().getTagPose(target.fiducialId);
      if (tagPose.isEmpty()) continue;

      avgDist += tagPose.get().toPose2d().getTranslation()
          .getDistance(estimate.estimatedPose.toPose2d().getTranslation());
      validTags++;
    }

    if (validTags == 0) {
      return k_ignorestddevs;
    }

    avgDist /= validTags;

    // Reject single-tag estimates beyond 10m
    if (numTags == 1 && avgDist > 10.0) {
      return k_ignorestddevs;
    }

    Matrix<N3, N1> stddevs = (numTags > 1) ? k_multitagstddevs : k_singletagstddevs;

    // Scale by distance squared (official PhotonVision heuristic)
    return stddevs.times(1 + (avgDist * avgDist / 30.0));
  }

   // manual pose estimation
  public Optional<EstimatedRobotPose> manualEstimate(PhotonPipelineResult result) {
    if (result.targets.isEmpty()) return Optional.empty();

    // Pick the target with lowest ambiguity
    PhotonTrackedTarget bestTarget = null;
    double bestAmbiguity = Double.MAX_VALUE;
    for (var target : result.targets) {
      if (target.getPoseAmbiguity() < bestAmbiguity
          && k_fieldlayout.getTagPose(target.fiducialId).isPresent()) {
        bestAmbiguity = target.getPoseAmbiguity();
        bestTarget = target;
      }
    }

    if (bestTarget == null) {
      return Optional.empty();
    }

    // tagPose on field - camera pose on field - robot pose on field
    Pose3d tagPose = k_fieldlayout.getTagPose(bestTarget.fiducialId).get();
    Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
    Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
    Pose3d robotPose = cameraPose.transformBy(m_robotToCamera.inverse());

    return Optional.of(new EstimatedRobotPose(
        robotPose,
        result.getTimestampSeconds(),
        List.of(bestTarget)));
  }

  // Returns the most recent pipeline result. */
  public Optional<PhotonPipelineResult> getResults() {
    return results;
  }

  // Returns the current estimate with standard deviations. */
  public Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>> getEstimate() {
    return estimate;
  }

  // Returns a boolean with the connection status of the camera.
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  // Returns the camera sim instance to interface with.
  public PhotonCameraSim getSimInstance() {
    return m_camerasim;
  }

  @Override
  public void periodic() {
    String p = "Vision/" + m_name + "/";
    results = Optional.empty();
    
    for (var result : m_camera.getAllUnreadResults()) {
      results = Optional.of(result);
    }

    int targetCount = results.map(r -> r.targets.size()).orElse(0);
    SmartDashboard.putBoolean(p + "Connected", m_camera.isConnected());
    SmartDashboard.putNumber(p + "TargetCount", targetCount);

    results.ifPresentOrElse(result -> {
      var est = m_estimator.estimateCoprocMultiTagPose(result);
      SmartDashboard.putBoolean(p + "MultiTagWorked", est.isPresent());

      if (est.isEmpty()) {
        est = m_estimator.estimateLowestAmbiguityPose(result);
        SmartDashboard.putBoolean(p + "LowestAmbiguityWorked", est.isPresent());
      }

      // manual pose from raw target transform 
      // if (est.isEmpty()) {
      //   est = manualEstimate(result);
      //   SmartDashboard.putBoolean(p + "ManualEstimateWorked", est.isPresent());
      // }

      est.ifPresentOrElse(pose -> {
        // estimate = Pair.of(Optional.of(pose), calculateStdDevs(pose, result));
        estimate = Pair.of(Optional.of(pose), k_multitagstddevs);

        // debug
        Pose2d robotPose2d = pose.estimatedPose.toPose2d();
        SmartDashboard.putNumber(p + "EstX", robotPose2d.getX());
        SmartDashboard.putNumber(p + "EstY", robotPose2d.getY());
        SmartDashboard.putNumber(p + "EstDeg", robotPose2d.getRotation().getDegrees());
        SmartDashboard.putNumber(p + "Timestamp", pose.timestampSeconds);
        SmartDashboard.putNumber(p + "TagsUsed", pose.targetsUsed.size());
      }, () -> {
        estimate = Pair.of(Optional.empty(), k_ignorestddevs);
      });

      SmartDashboard.putBoolean(p + "HasEstimate", estimate.getFirst().isPresent());

    }, () -> {
      estimate = Pair.of(Optional.empty(), k_ignorestddevs);
      SmartDashboard.putBoolean(p + "HasEstimate", false);
    });
  }
}
