package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * Factory class for teleop tunnel cycling commands.
 *
 * Each cycle command pathfinds through a tunnel as fast as possible,
 * starting from whichever end is closest. No pre-rotation step —
 * the robot drives immediately and lets PathPlanner handle heading.
 */
public final class CycleCommands {

    private CycleCommands() {}

    // ===== Blue Alliance Waypoints =====
    private static final Pose2d BLUE_LEFT_NEAR = new Pose2d(3.0, 7.4, Rotation2d.fromDegrees(0));
    private static final Pose2d BLUE_LEFT_FAR  = new Pose2d(7.0, 7.4, Rotation2d.fromDegrees(180));

    private static final Pose2d BLUE_RIGHT_NEAR = new Pose2d(3.0, 0.5, Rotation2d.fromDegrees(0));
    private static final Pose2d BLUE_RIGHT_FAR  = new Pose2d(7.0, 0.5, Rotation2d.fromDegrees(180));

    // ===== Red Alliance Waypoints =====
    private static final Pose2d RED_LEFT_NEAR = new Pose2d(13.25, 0.5, Rotation2d.fromDegrees(180));
    private static final Pose2d RED_LEFT_FAR  = new Pose2d(9.5, 0.5, Rotation2d.fromDegrees(0));

    private static final Pose2d RED_RIGHT_NEAR = new Pose2d(13.25, 7.4, Rotation2d.fromDegrees(180));
    private static final Pose2d RED_RIGHT_FAR  = new Pose2d(9.5, 7.4, Rotation2d.fromDegrees(0));

    // Pass-through velocity so the robot doesn't stop at the first waypoint
    private static final double PASS_THROUGH_VELOCITY = 2.0; // m/s

    // If already within this distance of the first waypoint, skip directly to crossing
    private static final double SKIP_DISTANCE = 1.5; // meters

    public static Command leftTunnelCycle(Swerve swerve, BooleanSupplier cancelCondition) {
        return defer(() -> {
            boolean isRed = isRedAlliance();
            Pose2d near = isRed ? RED_LEFT_NEAR : BLUE_LEFT_NEAR;
            Pose2d far  = isRed ? RED_LEFT_FAR  : BLUE_LEFT_FAR;
            return buildCycle(swerve, near, far);
        }, Set.of(swerve)).until(cancelCondition);
    }

    public static Command rightTunnelCycle(Swerve swerve, BooleanSupplier cancelCondition) {
        return defer(() -> {
            boolean isRed = isRedAlliance();
            Pose2d near = isRed ? RED_RIGHT_NEAR : BLUE_RIGHT_NEAR;
            Pose2d far  = isRed ? RED_RIGHT_FAR  : BLUE_RIGHT_FAR;
            return buildCycle(swerve, near, far);
        }, Set.of(swerve)).until(cancelCondition);
    }

    /**
     * Determines which waypoint is closer and pathfinds through the tunnel.
     * No alignment — drives immediately to the closer end, passes through it,
     * then continues to the far end.
     */
    private static Command buildCycle(Swerve swerve, Pose2d near, Pose2d far) {
        Pose2d robotPose = swerve.getPose();
        double distToNear = robotPose.getTranslation().getDistance(near.getTranslation());
        double distToFar  = robotPose.getTranslation().getDistance(far.getTranslation());

        if (distToNear <= distToFar) {
            Pose2d farNoRotate = new Pose2d(far.getTranslation(), near.getRotation());
            if (distToNear < SKIP_DISTANCE) {
                // Already at near — cross immediately, keep current heading
                Pose2d farKeepHeading = new Pose2d(far.getTranslation(), robotPose.getRotation());
                return swerve.pathfindToPose(farKeepHeading, false, 0.0);
            }
            return sequence(
                swerve.pathfindToPose(near, false, PASS_THROUGH_VELOCITY),
                swerve.pathfindToPose(farNoRotate, false, 0.0)
            );
        } else {
            Pose2d nearNoRotate = new Pose2d(near.getTranslation(), far.getRotation());
            if (distToFar < SKIP_DISTANCE) {
                // Already at far — cross immediately, keep current heading
                Pose2d nearKeepHeading = new Pose2d(near.getTranslation(), robotPose.getRotation());
                return swerve.pathfindToPose(nearKeepHeading, false, 0.0);
            }
            return sequence(
                swerve.pathfindToPose(far, false, PASS_THROUGH_VELOCITY),
                swerve.pathfindToPose(nearNoRotate, false, 0.0)
            );
        }
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
