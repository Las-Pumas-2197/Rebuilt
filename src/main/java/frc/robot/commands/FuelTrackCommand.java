package frc.robot.commands;

import static frc.robot.utils.Constants.VisionConstants.k_limelightname;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Detector;

/**
 * Drives toward the largest cluster of detected FUEL (balls) using the Limelight.
 * Uses area-weighted tx averaging so nearby dense groups pull harder.
 * Hold to track, release to stop.
 */
public class FuelTrackCommand extends Command {

    private static final int FUEL_CLASS_ID = 0;
    private static final double DRIVE_SPEED = 1.5;  // m/s forward
    private static final double STEER_KP = 0.03;
    private static final double MIN_AREA = 0.002;   // ignore tiny detections
    private static final double TX_DEADBAND = 25.0; // degrees — no correction within this range
    private static final double IDLE_RPS = 1.5;

    private final Swerve m_swerve;

    public FuelTrackCommand(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        LimelightResults results = LimelightHelpers.getLatestResults(k_limelightname);
        LimelightTarget_Detector[] detections = results.targets_Detector;

        double weightedTx = 0;
        double totalArea = 0;
        int fuelCount = 0;

        for (LimelightTarget_Detector det : detections) {
            if ((int) det.classID == FUEL_CLASS_ID && det.ta > MIN_AREA) {
                weightedTx += det.tx * det.ta;
                totalArea += det.ta;
                fuelCount++;
            }
        }

        SmartDashboard.putNumber("FuelTrack/Count", fuelCount);

        if (fuelCount == 0 || totalArea == 0) {
            // No fuel detected
            m_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, IDLE_RPS));
            return;
        }

        double avgTx = weightedTx / totalArea;
        // Only steer if the target is outside the acceptable range
        double rotSpeed = Math.abs(avgTx) > TX_DEADBAND ? -STEER_KP * avgTx : 0;

        SmartDashboard.putNumber("FuelTrack/AvgTx", avgTx);
        SmartDashboard.putNumber("FuelTrack/RotSpeed", rotSpeed);

        m_swerve.driveRobotRelative(new ChassisSpeeds(DRIVE_SPEED, 0, rotSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.driveRobotRelative(new ChassisSpeeds());
    }
}
