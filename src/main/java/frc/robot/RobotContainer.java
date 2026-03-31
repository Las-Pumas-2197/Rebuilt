// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.PathfindingConstants.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.commands.Autos;
import frc.robot.commands.CycleCommands;
import frc.robot.commands.FuelTrackCommand;
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
    private final CommandGenericHID m_joystick3 = new CommandGenericHID(2);

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
    private double slideTargetPos = 0.0;

    private double drivespeedmult = 4;

    public RobotContainer() {

        // start data logging
        DataLogManager.start();
        // Configure pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        m_swerve.runAutoBuilder();

        // Register PathPlanner named commands
        NamedCommands.registerCommand("Shoot", defer(() ->
            new ParallelCommandGroup(
                turretTrackHubLeadCommand(),
                runEnd(() -> m_feeder.runFeeder(), () -> m_feeder.stopAllFeeder(), m_feeder)
            ), Set.of(m_turret, m_feeder)));

        NamedCommands.registerCommand("ExtendHopper",
            runOnce(() -> slideTargetPos = m_hopper.getPositionSetpoints()[2]));
        NamedCommands.registerCommand("RetractHopper",
            runOnce(() -> slideTargetPos = m_hopper.getPositionSetpoints()[1]));
        NamedCommands.registerCommand("RunIntake",
            runEnd(() -> m_intake.runIntake(), () -> m_intake.stopRoller(), m_intake));

        // Configure autos
        // m_autochooser.setDefaultOption("square", Autos.simpleSquareAuto(m_swerve, m_turret));
        // m_autochooser.setDefaultOption("center auto",
        //     Autos.centerAuto(m_swerve, m_hopper, m_intake, m_feeder,
        //     this::turretTrackHubCommand,
        //     this::shakeCommand,
        //     () -> turretTargetVel = 0.62,
        //     () -> turretTargetVel = 0));

        // m_autochooser.addOption("center auto left", 
        //     Autos.centerAutoLeft(m_swerve, m_hopper, m_intake, m_feeder,
        //     this::turretTrackHubCommand,
        //     this::shakeCommand,
        //     () -> turretTargetVel = 0.62,
        //     () -> turretTargetVel = 0));

        m_autochooser.setDefaultOption("SimTest", AutoBuilder.buildAuto("SimTest"));
        m_autochooser.addOption("Right to Center to Bump", AutoBuilder.buildAuto("Right to Center to Bump"));
        m_autochooser.addOption("Left to Center to Depot", AutoBuilder.buildAuto("Left to Center to Depot"));
        m_autochooser.addOption("Right to Center Full", AutoBuilder.buildAuto("Right to Center Full"));
        m_autochooser.addOption("Left Anti Superduper", AutoBuilder.buildAuto("Left Anti Superduper"));
        m_autochooser.addOption("Middle to Depot", AutoBuilder.buildAuto("Middle to Depot"));

        // m_autochooser.addOption("Simtest", AutoBuilder.buildAuto("SimTest"));


        // m_autochooser.setDefaultOption("drive under tag 28", Autos.driveUnderTagAuto(m_swerve, 28));
        // m_autochooser.addOption("autoalign reef A", Autos.autoAlignReef(m_swerve, 18));
        SmartDashboard.putData(m_autochooser);

        // Default commands
        m_vision.setDefaultCommand(visionDefaultCommand());
        m_swerve.setDefaultCommand(swerveDefaultCommand());
        m_turret.setDefaultCommand(turretDefaultCommand());
        m_hopper.setDefaultCommand(hopperDefaultCommand());

        configureBindings();
    }

    private void configureBindings() {
        m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));

        // hopper slide bindings
        m_joystick3.button(11).whileTrue(runEnd(() -> m_hopper.extendSlide(), () -> m_hopper.stopSlide(), m_hopper));
        m_joystick3.button(12).whileTrue(runEnd(() -> m_hopper.retractSlide(), () -> m_hopper.stopSlide(), m_hopper));

        // intake roller
        m_joystick.rightTrigger().whileTrue(runEnd(() -> m_intake.runIntake(), () -> m_intake.stopRoller(), m_intake));
        m_joystick.a().whileTrue(runEnd(() -> m_intake.runEject(), () -> m_intake.stopRoller(), m_intake));

        // fuel tracking — drive toward detected balls
        // m_joystick.leftTrigger().whileTrue(new FuelTrackCommand(m_swerve));

        // cycle
        m_joystick.povLeft().onTrue(CycleCommands.leftTunnelCycle(m_swerve, this::hasManualDriveInput));
        m_joystick.povRight().onTrue(CycleCommands.rightTunnelCycle(m_swerve, this::hasManualDriveInput));

        m_joystick.leftTrigger().whileTrue(run(() -> m_swerve.driveRobotRelative(
            new ChassisSpeeds(
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * drivespeedmult,
                -MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * drivespeedmult,
                -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop))
            , m_swerve)
        );

        // conveyor commands
        // m_joystick2.y().whileTrue(runEnd(() -> m_hopper.runConveyor(), () -> m_hopper.stopConveyor(), m_hopper));

        // Zero slide encoder (for measuring travel distance)
        // m_joystick.y().onTrue(runOnce(() -> m_hopper.zeroSlideEncoder(),  m_hopper));
        
        // Toggle slide extend/retract by position
        // m_joystick2.b().onTrue(m_hopper.slideCommand());

        // Feed belt and kicker
        m_joystick3.button(8).whileTrue(runEnd(() -> m_feeder.runFeeder(), () -> m_feeder.stopAllFeeder(), m_feeder));
        // m_joystick2.start().onTrue(runOnce(() -> turretTargetVel = 0.1));
        // m_joystick2.back().onTrue(runOnce(() -> turretTargetVel = 0));
        new Trigger(() -> this.hasAimInput()).whileTrue(runEnd(() -> turretTargetPos = this.getAimHeading(), () -> turretTargetPos = 0));

        m_joystick3.button(3).toggleOnTrue(turretTrackHubLeadCommand());

        m_joystick.leftBumper().whileTrue(runEnd(() -> drivespeedmult = 1, () -> drivespeedmult = k_maxlinspeedteleop));
        m_joystick.rightBumper().whileTrue(runEnd(() -> drivespeedmult = k_maxlinspeedturbo, () -> drivespeedmult = k_maxlinspeedteleop));


        // Shake robot forward/backward
        // m_joystick.y().whileTrue(shakeCommand());

        m_joystick.x().onTrue(runOnce(() -> m_swerve.resetGyro()));

        // hopper positions
        // m_joystick3.button(5).onTrue(runOnce(() -> slideTargetPos = m_hopper.getPositionSetpoints()[0]));
        m_joystick3.button(6).onTrue(runOnce(() -> slideTargetPos = m_hopper.getPositionSetpoints()[1]));
        m_joystick3.button(7).whileTrue(sequence(
            runOnce(() -> slideTargetPos = m_hopper.getPositionSetpoints()[2]),
            runEnd(() -> m_intake.runEject(), () -> m_intake.stopRoller(), m_intake)
        ));

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
        cmd.addCommands(run(() -> {
            m_vision.updateFrontCameraSlideOffset(m_hopper.getSlideExtensionFraction());
            m_swerve.addVisionMeasurements(m_vision.getEstimates());
        }));
        cmd.addRequirements(m_vision);
        return cmd;
    }

    private Command swerveDefaultCommand() {
        return new
            RunCommand(() -> m_swerve.drive(new ChassisSpeeds(
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * drivespeedmult,
                -MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * drivespeedmult,
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

    private Command hopperDefaultCommand() {
        return new RunCommand(() -> m_hopper.slideCL(slideTargetPos), m_hopper);
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

    private Command turretTrackHubLeadCommand() {
        final double avgBallSpeed = 3.0; // m/s — tune 
        return new RunCommand(() -> {
            Pose2d robotPose = m_swerve.getPose();
            Pose2d targetPose = getTargetPose();
            ChassisSpeeds fieldSpeeds = m_swerve.getFieldSpeeds();
            double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            double flywheelSpeed = m_turret.interpolateFlywheelSpeed(distance);
            double leadAngle = MathUtil.angleModulus(
                m_turret.calculateLeadCorrectedAngle(robotPose, targetPose, fieldSpeeds, avgBallSpeed) + Math.PI);
            m_turret.turretCL(flywheelSpeed, leadAngle);
        }, m_turret);
    }

    public Command getAutonomousCommand() {
        return m_autochooser.getSelected();
    }

    /** Returns the turret target pose based on alliance and robot position.
     *  When past the midfield, aims at pass-back locations instead of the basin. */
    public Pose2d getTargetPose() {
        Pose2d robotPose = m_swerve.getPose();
        double x = robotPose.getX();
        double y = robotPose.getY();
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (isRed) {
            if (x < k_redMidfieldX) {
                return y > 4 ? k_redPassBackUpper : k_redPassBackLower;
            }
            return k_redBasinCenter;
        } else {
            if (x > k_blueMidfieldX) {
                return y > 4 ? k_bluePassBackUpper : k_bluePassBackLower;
            }
            return k_basinCenter;
        }
    }

    public void testRun() {
        SmartDashboard.putNumber("turret heading", turretTargetPos);
        Pose2d target = getTargetPose();
        SmartDashboard.putNumberArray("Turret Target",
            new double[] { target.getX(), target.getY(), target.getRotation().getDegrees() });
    }
}
