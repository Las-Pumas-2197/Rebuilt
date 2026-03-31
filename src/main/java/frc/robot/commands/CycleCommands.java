package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * Factory class for teleop tunnel cycling commands.
 *
 * Each cycle command drives the robot backwards through a tunnel,
 * alternating between a waypoint on each side. The robot heading
 * always faces away from the direction of travel (driving in reverse).
 */
public final class CycleCommands {

    private CycleCommands() {}

    // ===== Blue Alliance Waypoints =====

    // Left tunnel (blue side)
    private static final Pose2d BLUE_LEFT_NEAR = new Pose2d(3.0, 7.4, Rotation2d.fromDegrees(0));  
    private static final Pose2d BLUE_LEFT_FAR  = new Pose2d(7.0, 7.4, Rotation2d.fromDegrees(180));

    // Right tunnel (blue side)
    private static final Pose2d BLUE_RIGHT_NEAR = new Pose2d(3.0, 0.5, Rotation2d.fromDegrees(0)); 
    private static final Pose2d BLUE_RIGHT_FAR  = new Pose2d(7.0, 0.5, Rotation2d.fromDegrees(180));

    // ===== Red Alliance Waypoints =====

    // Left tunnel (red side)
    private static final Pose2d RED_LEFT_NEAR = new Pose2d(13.25, 0.5, Rotation2d.fromDegrees(180));
    private static final Pose2d RED_LEFT_FAR  = new Pose2d(9.5, 0.5, Rotation2d.fromDegrees(0));   

    // Right tunnel (red side)
    private static final Pose2d RED_RIGHT_NEAR = new Pose2d(13.25, 7.4, Rotation2d.fromDegrees(180));
    private static final Pose2d RED_RIGHT_FAR  = new Pose2d(9.5, 7.4, Rotation2d.fromDegrees(0));  

    /**
     * Creates a cycle command for the left tunnel.
     * Robot drives backwards from near waypoint through tunnel to far waypoint, then back.
     * Loops until cancelled or cancelCondition is true.
     */
    public static Command leftTunnelCycle(Swerve swerve, BooleanSupplier cancelCondition) {
        return defer(() -> {
            boolean isRed = isRedAlliance();
            Pose2d near = isRed ? RED_LEFT_NEAR : BLUE_LEFT_NEAR;
            Pose2d far  = isRed ? RED_LEFT_FAR  : BLUE_LEFT_FAR;
            return buildCycle(swerve, near, far, isRed);
        }, Set.of(swerve)).until(cancelCondition);
    }

    /**
     * Creates a cycle command for the right tunnel.
     * Determines which end is closer, aligns, drives to it, then crosses. Runs once.
     * Cancels if cancelCondition is true.
     */
    public static Command rightTunnelCycle(Swerve swerve, BooleanSupplier cancelCondition) {
        return defer(() -> {
            boolean isRed = isRedAlliance();
            Pose2d near = isRed ? RED_RIGHT_NEAR : BLUE_RIGHT_NEAR;
            Pose2d far  = isRed ? RED_RIGHT_FAR  : BLUE_RIGHT_FAR;
            return buildCycle(swerve, near, far, isRed);
        }, Set.of(swerve)).until(cancelCondition);
    }

    /**
     * Determines which waypoint is closer, aligns to the crossing orientation,
     * drives to the closest waypoint, then crosses to the other.
     *
     * Red: near->far = 180°, far->near = 0°
     * Blue: near->far = 0°, far->near = 180°
     */
    private static final double SKIP_WAYPOINT_DIST = 1.5; // meters

    private static Command buildCycle(Swerve swerve, Pose2d near, Pose2d far, boolean isRed) {
        Pose2d robotPose = swerve.getPose();
        double distToNear = robotPose.getTranslation().getDistance(near.getTranslation());
        double distToFar = robotPose.getTranslation().getDistance(far.getTranslation());

        if (distToNear <= distToFar) {
            // Closer to near, will cross near -> far
            Rotation2d heading = Rotation2d.fromDegrees(isRed ? 180 : 0);
            if (distToNear < SKIP_WAYPOINT_DIST) {
                // Already at near, just align and cross
                return sequence(
                    alignToHeading(swerve, heading),
                    swerve.pathfindToPose(new Pose2d(far.getTranslation(), heading), false, 0.0)
                );
            }
            // Pathfind to near with correct heading (aligns while driving), then cross
            return sequence(
                swerve.pathfindToPose(new Pose2d(near.getTranslation(), heading), false, 1.0),
                swerve.pathfindToPose(new Pose2d(far.getTranslation(), heading), false, 0.0)
            );
        } else {
            // Closer to far, will cross far -> near
            Rotation2d heading = Rotation2d.fromDegrees(isRed ? 0 : 180);
            if (distToFar < SKIP_WAYPOINT_DIST) {
                // Already at far, just align and cross
                return sequence(
                    alignToHeading(swerve, heading),
                    swerve.pathfindToPose(new Pose2d(near.getTranslation(), heading), false, 0.0)
                );
            }
            // Pathfind to far with correct heading (aligns while driving), then cross
            return sequence(
                swerve.pathfindToPose(new Pose2d(far.getTranslation(), heading), false, 1.0),
                swerve.pathfindToPose(new Pose2d(near.getTranslation(), heading), false, 0.0)
            );
        }
    }

    private static final double ALIGN_TOLERANCE = Math.toRadians(5);
    private static final double ALIGN_KP = 2.0;

    /** Rotates the robot in place to the target heading using proportional control. */
    private static Command alignToHeading(Swerve swerve, Rotation2d targetHeading) {
        return run(() -> {
            double error = MathUtil.angleModulus(
                targetHeading.getRadians() - swerve.getGyroHeading().getRadians());
            double rotSpeed = ALIGN_KP * error;
            swerve.drive(new ChassisSpeeds(0, 0, rotSpeed));
        }, swerve).until(() -> {
            double error = MathUtil.angleModulus(
                targetHeading.getRadians() - swerve.getGyroHeading().getRadians());
            return Math.abs(error) < ALIGN_TOLERANCE;
        }).withTimeout(3);
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}
