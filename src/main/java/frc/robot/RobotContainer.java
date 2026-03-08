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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.utils.Telemetry;

public class RobotContainer {

    // Controllers
    private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);
    private final CommandXboxController m_joystick2 = new CommandXboxController(1);

    // Subsystems
    private final Vision m_vision = new Vision();
    private final Swerve m_swerve = new Swerve();
    private final Hopper m_hopper = new Hopper();
    private final Intake m_intake = new Intake();
    private final Turret m_turret = new Turret();
    private final Feeder m_feeder = new Feeder();
    private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

    // Auto chooser
    private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

    private double turretTargetPos = 0.0;
    private double turretTargetVel = 0.0;

    public RobotContainer() {

        // start data logging
        DataLogManager.start();
        // Configure pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        m_swerve.runAutoBuilder();

        // Configure autos
        // m_autochooser.setDefaultOption("square", Autos.simpleSquareAuto(m_swerve, m_turret));
        m_autochooser.setDefaultOption("center auto",
            Autos.centerAuto(m_swerve, m_hopper, m_intake, m_feeder,
            this::turretTrackHubCommand,
            () -> turretTargetVel = 0.55,
            () -> turretTargetVel = 0));

        m_autochooser.addOption("center auto left", 
            Autos.centerAutoLeft(m_swerve, m_hopper, m_intake, m_feeder,
            this::turretTrackHubCommand,
            () -> turretTargetVel = 0.55,
            () -> turretTargetVel = 0));
        // m_autochooser.setDefaultOption("drive under tag 28", Autos.driveUnderTagAuto(m_swerve, 28));
        // m_autochooser.addOption("autoalign reef A", Autos.autoAlignReef(m_swerve, 18));
        SmartDashboard.putData(m_autochooser);

        // Default commands
        m_vision.setDefaultCommand(visionDefaultCommand());
        m_swerve.setDefaultCommand(swerveDefaultCommand());
        m_turret.setDefaultCommand(turretDefaultCommand());

        configureBindings();
    }

    private void configureBindings() {
        m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));
        // m_joystick.a().onTrue(CycleCommands.createCycleCommand(m_swerve, m_turret, this::hasManualDriveInput));

        // hopper slide bindings
        m_joystick2.povUp().whileTrue(runEnd(() -> m_hopper.extendSlide(), () -> m_hopper.stopSlide(), m_hopper));
        m_joystick2.povDown().whileTrue(runEnd(() -> m_hopper.retractSlide(), () -> m_hopper.stopSlide(), m_hopper));

        // intake roller
        m_joystick.rightTrigger().whileTrue(runEnd(() -> m_intake.runIntake(), () -> m_intake.stopRoller(), m_intake));

        // cycle
        m_joystick.povLeft().onTrue(CycleCommands.leftTunnelCycle(m_swerve, this::hasManualDriveInput));
        m_joystick.povRight().onTrue(CycleCommands.rightTunnelCycle(m_swerve, this::hasManualDriveInput));

        // Zero slide encoder (for measuring travel distance)
        // m_joystick.y().onTrue(runOnce(() -> m_hopper.zeroSlideEncoder(),  m_hopper));
        
        // Toggle slide extend/retract by position
        m_joystick2.b().onTrue(m_hopper.autoExtendSlide());

        // Feed belt and kicker
        m_joystick2.rightTrigger().whileTrue(runEnd(() -> m_feeder.runFeeder(), () -> m_feeder.stopAllFeeder(), m_feeder));
        m_joystick2.start().onTrue(runOnce(() -> turretTargetVel = 0.5));
        m_joystick2.back().onTrue(runOnce(() -> turretTargetVel = 0));
        new Trigger(() -> this.hasAimInput()).whileTrue(runEnd(() -> turretTargetPos = this.getAimHeading(), () -> turretTargetPos = 0));

        m_joystick2.leftTrigger().whileTrue(turretTrackHubCommand());


        // Shake robot forward/backward
        m_joystick.y().whileTrue(shakeCommand());

        m_joystick.x().onTrue(runOnce(() -> m_swerve.resetGyro()));

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

    private boolean hasAimInput() {
        return Math.abs(m_joystick2.getLeftY()) > 0.2
            || Math.abs(m_joystick2.getLeftX()) > 0.2;
    }

    private double getAimHeading() {
        return Math.atan2(-m_joystick2.getLeftX(), -m_joystick2.getLeftY());
    }

    private ParallelCommandGroup visionDefaultCommand() {
        ParallelCommandGroup cmd = new ParallelCommandGroup();
            if (RobotBase.isSimulation())
        cmd.addCommands(run(() -> m_vision.updateSimPose(m_swerve.getSimPose())));
        cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
        cmd.addRequirements(m_vision);
        return cmd;
    }

    private Command swerveDefaultCommand() {
        return new
            RunCommand(() -> m_swerve.drive(new ChassisSpeeds(
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * (m_joystick.rightBumper().getAsBoolean() ? k_maxlinspeedturbo : k_maxlinspeedteleop),
                -MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * (m_joystick.rightBumper().getAsBoolean() ? k_maxlinspeedturbo : k_maxlinspeedteleop),
                -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
            m_swerve);
    }

    private Command turretDefaultCommand() {
        return new RunCommand(
            () -> m_turret.turretCL(
                turretTargetVel,
                turretTargetPos
        ), m_turret);
    }

    private Command shakeCommand() {
        final double shakeSpeed = 5.5; // m/s
        final double shakeFrequency = 5.0; 
        return new RunCommand(() -> {
            double direction = Math.sin(Timer.getFPGATimestamp() * shakeFrequency * 2 * Math.PI) > 0 ? 1 : -1;
            m_swerve.drive(new ChassisSpeeds(direction * shakeSpeed, 0, 0));
        }, m_swerve);
    }

    private Command turretTrackHubCommand() {
        return new RunCommand(() -> {
            Pose2d robotPose = m_swerve.getPose();
            Pose2d targetPose = getTargetPose();
            double angleToHub = MathUtil.angleModulus(m_turret.calculateAngleToFieldPose(robotPose, targetPose) + Math.PI);
            m_turret.turretCL(0.5, angleToHub);
        }, m_turret);
    }

    public Command getAutonomousCommand() {
        return m_autochooser.getSelected();
    }

    /** Returns the turret target pose based on the current alliance. */
    public Pose2d getTargetPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red ? k_redBasinCenter : k_basinCenter;
        } else { 
            return k_basinCenter;
        }
    }

    public void testRun() {
        SmartDashboard.putNumber("turret heading", turretTargetPos);
    }
}
