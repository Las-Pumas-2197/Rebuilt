// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.PathfindingConstants.k_basinCenter;
import static frc.robot.utils.Constants.PathfindingConstants.k_redBasinCenter;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.commands.Autos;
import frc.robot.commands.CycleCommands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.BlackBox;
import frc.robot.utils.Telemetry;

public class RobotContainer {

    // Controllers
    private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
    private final CommandXboxController m_joystick2 = new CommandXboxController(1);

    // Subsystems
    private final Swerve m_swerve = new Swerve();
    private final Hopper m_hopper = new Hopper();
    private final Intake m_intake = new Intake();
    private final Turret m_turret = new Turret();
    private final Feeder m_feeder = new Feeder();
    private final Vision m_vision = new Vision(() -> m_swerve.getGyroHeading().getDegrees());
    private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

    // Alliance chooser — determines turret tracking target
    private final SendableChooser<Pose2d> m_allianceChooser = new SendableChooser<>();

    // Auto chooser
    private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

    private double turretTargetPos = 0.0;
    private double turretTargetVel = 0.0;

    public RobotContainer() {
        // Configure pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        m_swerve.runAutoBuilder();

        // Alliance selector for turret target
        m_allianceChooser.setDefaultOption("Blue", k_basinCenter);
        m_allianceChooser.addOption("Red", k_redBasinCenter);
        SmartDashboard.putData("Alliance", m_allianceChooser);

        // Configure autos
        // m_autochooser.setDefaultOption("square", Autos.simpleSquareAuto(m_swerve, m_turret));
        // m_autochooser.setDefaultOption("drive under tag 28", Autos.driveUnderTagAuto(m_swerve, 28));
        // m_autochooser.addOption("autoalign reef A", Autos.autoAlignReef(m_swerve, 18));
        m_autochooser.setDefaultOption("red pickup", Autos.redPickupAuto(m_swerve, m_hopper, m_intake));
        m_autochooser.addOption("blue pickup", Autos.bluePickupAuto(m_swerve, m_hopper, m_intake));
        
        SmartDashboard.putData(m_autochooser);

        // Default commands
        m_swerve.setDefaultCommand(swerveDefaultCommand());
        m_turret.setDefaultCommand(turretDefaultCommand());
        m_vision.setDefaultCommand(visionDefaultCommand());

        configureBindings();
    }

    private void configureBindings() {
        m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));

        // hopper slide bindings (slide motor now in Hopper)
        m_joystick.povUp().whileTrue(runEnd(() -> m_hopper.extendSlide(), () -> m_hopper.stopSlide(), m_hopper));
        m_joystick.povDown().whileTrue(runEnd(() -> m_hopper.retractSlide(), () -> m_hopper.stopSlide(), m_hopper));

        // intake roller
        m_joystick.a().toggleOnTrue(runEnd(() -> m_intake.runIntake(), () -> m_intake.stopRoller(), m_intake));

        // Toggle slide extend/retract by position
        m_joystick.b().onTrue(m_hopper.slideCommand());

        // Feed belt and kicker
        m_joystick2.rightTrigger().whileTrue(runEnd(() -> m_feeder.runFeeder(), () -> m_feeder.stopAllFeeder(), m_feeder));
        m_joystick2.start().onTrue(runOnce(() -> turretTargetVel = 0.1));
        m_joystick2.back().onTrue(runOnce(() -> turretTargetVel = 0));
        m_joystick2.axisGreaterThan(0, 0.2).whileTrue(run(() -> turretTargetPos = Math.atan2(-m_joystick2.getLeftX(), m_joystick2.getLeftY())));


        // turret
        // m_joystick.rightTrigger().whileTrue(runEnd(() -> m_turret.spinUpFlywheels(), () -> m_turret.stopAllShooter(), m_turret));

        // m_joystick.start().onTrue(runOnce(() -> m_turret.setFlywheelSpeed(0.5), m_turret));
        // m_joystick.back().onTrue(runOnce(() -> m_turret.setFlywheelSpeed(0), m_turret));

        // // turret manual rotation — nudges target yaw so PID+limits stay in control
        // m_joystick.povLeft().whileTrue(run(() -> m_turret.nudgeTargetYaw(-Math.toRadians(1.5)), m_turret));
        // m_joystick.povRight().whileTrue(run(() -> m_turret.nudgeTargetYaw(Math.toRadians(1.5)), m_turret));

        // m_joystick.leftBumper().onTrue(runOnce(() -> turretTargetPos = -0.6*Math.PI));
        // m_joystick.rightBumper().onTrue(runOnce(() -> turretTargetPos = 0));

        // new Trigger(() -> RobotState.isEnabled()).whileTrue(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
    }

    private boolean hasManualDriveInput() {
        final double deadband = 0.2;
        return Math.abs(m_joystick.getLeftX()) > deadband
            || Math.abs(m_joystick.getLeftY()) > deadband
            || Math.abs(m_joystick.getRightX()) > deadband;
    }

    private ParallelCommandGroup visionDefaultCommand() {
        ParallelCommandGroup cmd = new ParallelCommandGroup();
        if (RobotBase.isSimulation())
            cmd.addCommands(run(() -> m_vision.updatePose(m_swerve.getSimPose())));
        cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
        cmd.addRequirements(m_vision);
        return cmd;
    }

    private ParallelCommandGroup swerveDefaultCommand() {
        return new ParallelCommandGroup (
            run(() -> m_swerve.drive(new ChassisSpeeds(
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
            m_swerve)
            // run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates()))
            );
    }

    private Command turretDefaultCommand() {
        return new RunCommand(
            () -> m_turret.turretCL(
                turretTargetVel,
                turretTargetPos
        ), m_turret);
    }

    // Tracks the hub pose from the alliance chooser with distance-based flywheel speed
    private Command turretTrackHubCommand() {
        return new RunCommand(() -> {
            Pose2d robotPose = m_swerve.getPose();
            Pose2d targetPose = getTargetPose();
            double angleToHub = m_turret.calculateAngleToFieldPose(robotPose, targetPose);
            double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            double flywheelSpeed = m_turret.interpolateFlywheelSpeed(distance);
            m_turret.turretCL(flywheelSpeed, angleToHub);
        }, m_turret);
    }



    public Command getAutonomousCommand() {
        return m_autochooser.getSelected();
    }

    /** Returns the turret target pose based on the alliance chooser. */
    public Pose2d getTargetPose() {
        return m_allianceChooser.getSelected();
    }

    public void testRun() {}
}
