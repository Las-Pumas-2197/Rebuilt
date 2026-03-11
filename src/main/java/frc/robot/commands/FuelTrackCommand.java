package frc.robot.commands;

import static frc.robot.utils.Constants.VisionConstants.k_limelightname;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawDetection;

/**
 * Drives toward the largest cluster of detected FUEL (balls) using the Limelight.
 * Uses area-weighted tx averaging so nearby dense groups pull harder.
 * Hold to track, release to stop.
 */
public class FuelTrackCommand extends Command {

    private static final int FUEL_CLASS_ID = 0;
    private static final double DRIVE_SPEED = 0.1;  // m/s forward
    private static final double STEER_KP = 0.04;    // proportional gain on tx (degrees -> rad/s)
    private static final double MIN_AREA = 0.005;   // ignore tiny detections

    private final Swerve m_swerve;

    public FuelTrackCommand(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        RawDetection[] detections = LimelightHelpers.getRawDetections(k_limelightname);

        double weightedTx = 0;
        double totalArea = 0;
        int fuelCount = 0;

        for (RawDetection det : detections) {
            if (det.classId == FUEL_CLASS_ID && det.ta > MIN_AREA) {
                weightedTx += det.txnc * det.ta;
                totalArea += det.ta;
                fuelCount++;
            }
        }

        SmartDashboard.putNumber("FuelTrack/Count", fuelCount);

        if (fuelCount == 0 || totalArea == 0) {
            // No fuel detected, drive forward slowly
            m_swerve.drive(new ChassisSpeeds(DRIVE_SPEED * 0.5, 0, 0));
            return;
        }

        double avgTx = weightedTx / totalArea;
        double rotSpeed = -STEER_KP * avgTx;

        SmartDashboard.putNumber("FuelTrack/AvgTx", avgTx);
        SmartDashboard.putNumber("FuelTrack/RotSpeed", rotSpeed);

        m_swerve.drive(new ChassisSpeeds(DRIVE_SPEED, 0, rotSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new ChassisSpeeds());
    }
}
