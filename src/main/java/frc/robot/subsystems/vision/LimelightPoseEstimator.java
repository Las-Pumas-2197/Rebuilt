package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Drop-in Limelight-based pose estimator using two Limelights.
 * Runs independently of the PhotonVision pipeline so the turret
 * always has a pose source even when coprocessors drop out.
 *
 * Usage in RobotContainer:
 *   private final LimelightPoseEstimator m_llPose = new LimelightPoseEstimator(
 *       "limelight-left", "limelight-right", () -> m_swerve.getHeading().getDegrees());
 *
 * Then in visionDefaultCommand or a trigger:
 *   m_swerve.addVisionMeasurement(m_llPose.getEstimatedPose(), m_llPose.getTimestamp(), m_llPose.getStdDevs());
 *
 * Or for turret-only tracking, read m_llPose.getEstimatedPose() instead of m_swerve.getPose().
 */
public class LimelightPoseEstimator extends SubsystemBase {

    private final String m_llNameA;
    private final DoubleSupplier m_gyroYawDeg;

    // Latest fused estimate
    private Optional<Pose2d> m_estimatedPose = Optional.empty();
    private Matrix<N3, N1> m_stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private double m_timestamp = 0.0;

    // Tuning — base std devs
    private static final Matrix<N3, N1> SINGLE_TAG_STD = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    private static final Matrix<N3, N1> MULTI_TAG_STD  = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
    private static final Matrix<N3, N1> IGNORE_STD     = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    // Reject estimates with avg tag distance beyond this
    private static final double MAX_TAG_DISTANCE = 6.0; // m

    /**
     * @param llNameA  NetworkTables name of the first Limelight (e.g. "limelight-left")
     * @param llNameB  NetworkTables name of the second Limelight (e.g. "limelight-right")
     * @param gyroYawDeg  Supplier for the robot's gyro yaw in degrees (for MegaTag2)
     */
    public LimelightPoseEstimator(String llNameA, DoubleSupplier gyroYawDeg) {
        m_llNameA = llNameA;
        m_gyroYawDeg = gyroYawDeg;
    }

    /** Returns the latest fused pose estimate, or empty if no valid data. */
    public Optional<Pose2d> getEstimatedPose() {
        return m_estimatedPose;
    }

    /** Returns the std devs of the latest estimate. */
    public Matrix<N3, N1> getStdDevs() {
        return m_stdDevs;
    }

    /** Returns the timestamp of the latest estimate (FPGA seconds). */
    public double getTimestamp() {
        return m_timestamp;
    }

    @Override
    public void periodic() {
        // Feed gyro heading to both Limelights for MegaTag2
        double yaw = m_gyroYawDeg.getAsDouble();
        LimelightHelpers.SetRobotOrientation(m_llNameA, yaw, 0, 0, 0, 0, 0);

        // Pull MegaTag2 estimates from both cameras
        PoseEstimate estA = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_llNameA);

        boolean validA = isValid(estA);

        SmartDashboard.putBoolean("LL/" + m_llNameA + "/Valid", validA);

         if (validA) {
            m_estimatedPose = Optional.of(estA.pose);
            m_timestamp = estA.timestampSeconds;
            m_stdDevs = calculateStdDevs(estA.tagCount, estA.avgTagDist);
        } else {
            m_estimatedPose = Optional.empty();
            m_stdDevs = IGNORE_STD;
        }

        // Telemetry
        m_estimatedPose.ifPresent(pose -> SmartDashboard.putNumberArray("LL/EstimatedPose",
            new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }));
        SmartDashboard.putNumber("LL/StdDevX", m_stdDevs.get(0, 0));
        SmartDashboard.putNumber("LL/StdDevY", m_stdDevs.get(1, 0));
    }

    private boolean isValid(PoseEstimate est) {
        if (est == null) return false;
        if (est.tagCount == 0) return false;
        if (est.avgTagDist > MAX_TAG_DISTANCE) return false;
        // Reject poses clearly off-field
        if (est.pose.getX() < -1 || est.pose.getX() > 17
            || est.pose.getY() < -1 || est.pose.getY() > 9) return false;
        return true;
    }

    private Matrix<N3, N1> calculateStdDevs(int tagCount, double avgDist) {
        Matrix<N3, N1> base = (tagCount > 1) ? MULTI_TAG_STD : SINGLE_TAG_STD;
        double distFactor = Math.max(1.0, Math.pow(avgDist / 2.0, 2));
        double tagFactor = 1.0 / Math.sqrt(tagCount);
        return base.times(distFactor * tagFactor);
    }
}
