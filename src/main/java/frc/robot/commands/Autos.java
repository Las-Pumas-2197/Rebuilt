package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.PathfindingConstants.*;
import static frc.robot.utils.Constants.SwerveDriveConstants.k_initpose;
import static frc.robot.utils.Constants.VisionConstants.k_fieldlayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;

/**
 * Factory class for autonomous command routines.
 */
public final class Autos {

    private Autos() {}

    /**
     * Auto that cycles through a tag waypoint to the field center and back.
     * Path: Start -> Under Tag -> Field Center -> Under Tag -> Start
     * Robot passes through waypoints continuously without stopping.
     */
    public static Command driveUnderTagAuto(Swerve swerve, int tagId) {
        var tagPose = k_fieldlayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return print("ERROR: Tag " + tagId + " not found in field layout");
        }

        Pose2d waypointPose = new Pose2d(
            tagPose.get().getX(),
            tagPose.get().getY(),
            new Rotation2d(0)
        );

        Pose2d fieldCenterPose = k_fieldCenter;
        final double waypointVelocity = 2.0;

        return sequence(
            defer(() -> swerve.pathfindToPose(
                new Pose2d(waypointPose.getX(), waypointPose.getY(), swerve.getPose().getRotation()),
                true,
                waypointVelocity
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(fieldCenterPose.getX(), fieldCenterPose.getY(), swerve.getPose().getRotation()),
                true,
                0.0
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(waypointPose.getX(), waypointPose.getY(), swerve.getPose().getRotation()),
                true,
                waypointVelocity
            ), java.util.Set.of(swerve)),
            defer(() -> swerve.pathfindToPose(
                new Pose2d(k_initpose.getX(), k_initpose.getY(), swerve.getPose().getRotation()),
                true,
                0.0
            ), java.util.Set.of(swerve))
        );
    }

    /** Simple drive test - no pathfinding, just drives forward for 2 seconds. */
    public static Command simpleDriveTest(Swerve swerve) {
        return run(() -> swerve.drive(new ChassisSpeeds(1.0, 0, 0)), swerve)
            .withTimeout(2.0);
    }

    /** Simple square pattern auto with turret tracking. */
    public static Command simpleSquareAuto(Swerve swerve, Turret turret) {
        SequentialCommandGroup driveSequence = new SequentialCommandGroup(
            swerve.pathfindToPose(k_squarePoint1, false),
            swerve.pathfindToPose(k_squarePoint2, false),
            swerve.pathfindToPose(k_squarePoint3, false),
            swerve.pathfindToPose(k_squarePoint4, false));

        // Track the basin center during auto
        Command turretTrackingCommand = run(() -> {
            turret.aimAtFieldPose(swerve.getPose(), k_basinCenter);
        });

        return new ParallelDeadlineGroup(driveSequence, turretTrackingCommand);
    }

    /** Autoalign sequence for the reef. */
    public static Command autoAlignReef(Swerve swerve, int fiducialID) {
        return new SequentialCommandGroup(
            swerve.pathfindToPose(k_bluereefA, true),
            swerve.pathfindToPose(k_redreefA, true));
    }

    /**
     * Blue side auto: deploy hopper slide, drive across, intake while driving to
     * field center, then return to start.
     */
    public static Command bluePickupAuto(Swerve swerve, Hopper hopper, Intake intake) {
        Pose2d startPose = new Pose2d(3.5, 0.5, new Rotation2d(0));
        Pose2d farPose = new Pose2d(8, 0.5, new Rotation2d(Units.degreesToRadians(90)));
        Pose2d returnPose = new Pose2d(3.5, 0.5, new Rotation2d(Units.degreesToRadians(180)));
        Pose2d centerPose = k_fieldCenter;

        return sequence(
            // 1. Run hopper slide command (extend)
            hopper.slideCommand(),
            // 2. Drive to (8, 0.5), turning 90° left
            defer(() -> swerve.pathfindToPose(farPose, true, 0.0), java.util.Set.of(swerve)),
            // 3. Run intake while driving to field center
            new ParallelDeadlineGroup(
                defer(() -> swerve.pathfindToPose(
                    new Pose2d(centerPose.getX(), centerPose.getY(), swerve.getPose().getRotation()),
                    true, 0.0
                ), java.util.Set.of(swerve)),
                runEnd(intake::runIntake, intake::stopRoller, intake)
            ),
            // 4. Drive back to (8, 0.5) rotated 90° left
            defer(() -> swerve.pathfindToPose(new Pose2d(8, 0.5, new Rotation2d(Units.degreesToRadians(180))), true, 1.0), java.util.Set.of(swerve)),
            // 5. Return to start facing 180° (opposite of starting heading)
            defer(() -> swerve.pathfindToPose(returnPose, true, 0.0), java.util.Set.of(swerve))
        );
    }

    /**
     * Red side auto: mirror of bluePickupAuto across field center (x mirrored around 8.1).
     */
    public static Command redPickupAuto(Swerve swerve, Hopper hopper, Intake intake) {
        Pose2d farPose = new Pose2d(8.2, 7.5, new Rotation2d(Units.degreesToRadians(-90)));
        Pose2d returnPose = new Pose2d(12.7, 7.5, new Rotation2d(0));
        Pose2d centerPose = k_fieldCenter;

        return sequence(
            // 1. Run hopper slide command (extend)
            hopper.slideCommand(),
            // 2. Drive to (8.2, 0.5), turning 90° left
            defer(() -> swerve.pathfindToPose(farPose, true, 0.0), java.util.Set.of(swerve)),
            // 3. Run intake while driving to field center
            new ParallelDeadlineGroup(
                defer(() -> swerve.pathfindToPose(
                    new Pose2d(centerPose.getX(), centerPose.getY(), swerve.getPose().getRotation()),
                    true, 0.0
                ), java.util.Set.of(swerve)),
                runEnd(intake::runIntake, intake::stopRoller, intake)
            ),
            // 4. Drive back to (8.2, 0.5) facing back toward red side
            defer(() -> swerve.pathfindToPose(new Pose2d(8.2, 7.5, new Rotation2d(0)), true, 1.0), java.util.Set.of(swerve)),
            // 5. Return to start facing 0° (opposite of starting 180° heading)
            defer(() -> swerve.pathfindToPose(returnPose, true, 0.0), java.util.Set.of(swerve))
        );
    }
}
