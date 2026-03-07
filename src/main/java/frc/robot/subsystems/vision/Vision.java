// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {

  private final List<VisionCamera> m_cameras = new ArrayList<>();
  // private final LimelightCamera m_limelight;

  public Vision(DoubleSupplier yawDegreesSupplier) {
    for (int num = 0; num < k_cameranames.size(); num++) {
      m_cameras.add(new VisionCamera(k_cameranames.get(num), k_cameraintrinsics.get(num)));
    }
    // m_limelight = new LimelightCamera(k_limelightname, yawDegreesSupplier);
  }

  /** Get a list of all of the estimates from all camera pipelines (PhotonVision + Limelight). */
  public List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> getEstimates() {
    List<Pair<Optional<EstimatedRobotPose>, Matrix<N3, N1>>> estimates = new ArrayList<>();
    estimates.clear();
    for (var camera : m_cameras) {
      if (camera.getEstimate().getFirst().isPresent()) {
        estimates.add(camera.getEstimate());
      }
    }
    // var llEstimate = m_limelight.getEstimate();
    // if (llEstimate.getFirst().isPresent()) {
    //   estimates.add(llEstimate);
    // }
    return estimates;
  }

  /** Get a list of all the results from the camera pipelines. */
  public List<Optional<PhotonPipelineResult>> getResults() {
    List<Optional<PhotonPipelineResult>> results = new ArrayList<>();
    for (var camera : m_cameras) {
      if (camera.getResults().isPresent()) {
        results.add(camera.getResults());
      }
    }
    return results;
  }

  /** Returns a list containing all the visible fiducial IDs. */
  public List<Integer> getAllFiducialIDs() {
    HashSet<Integer> fiducials = new HashSet<>();
    for (var camera : m_cameras) {
      fiducials.addAll(camera.getFiducialIDs());
    }
    return fiducials.stream().toList();
  }


  public Rotation2d getCameraTargetYaw(int fiducialID, int cameraID) {
    Optional<PhotonPipelineResult> result = m_cameras.get(cameraID).getResults();
    Rotation2d target_yaw = new Rotation2d();
    if (result.isPresent()) {
      for (var target : result.get().targets) {
        if (target.fiducialId == fiducialID) {
          target_yaw = Rotation2d.fromDegrees(target.getYaw());
        }
      }
    }
    return target_yaw;
  }

  public double getCameraTargetDistance(int fiducialID, int cameraID) {
    Optional<PhotonPipelineResult> result = m_cameras.get(cameraID).getResults();
    double target_distance = 0;
    if (result.isPresent()) {
      for (var target : result.get().targets) {
        if (target.fiducialId == fiducialID) {
          target_distance = target.getBestCameraToTarget().getTranslation()
              .plus(k_cameraintrinsics.get(cameraID).getTranslation())
              .getDistance(new Translation3d());
        }
      }
    }
    return target_distance;
  }

  /** Returns a list of the vision system camera instances. */
  public List<VisionCamera> getCameras() {
    return m_cameras;
  }

  @Override
  public void periodic() {
    // Per-camera summary
    for (int i = 0; i < m_cameras.size(); i++) {
      var cam = m_cameras.get(i);
      SmartDashboard.putBoolean("Vision/cam" + i + "/HasResults", cam.getResults().isPresent());
      SmartDashboard.putBoolean("Vision/cam" + i + "/HasEstimate", cam.getEstimate().getFirst().isPresent());
    }

    // How many estimates will reach addVisionMeasurements
    var estimates = getEstimates();
    SmartDashboard.putNumber("Vision/EstimatesPassedToSwerve", estimates.size());
  }
}
