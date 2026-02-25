// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.PathfindingConstants.k_basinCenter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.commands.Autos;
import frc.robot.commands.CycleCommands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.BlackBox;
import frc.robot.utils.Telemetry;

public class RobotContainer {

    // Controllers
    private final CommandXboxController m_joystick = new CommandXboxController(k_joystickport);

    // Subsystems
    private final Swerve m_swerve = new Swerve();
    private final Vision m_vision = new Vision();
    private final Turret m_turret = new Turret();
    private final Telemetry m_telemetry = new Telemetry(m_vision, m_swerve);

    // Auto chooser
    private final SendableChooser<Command> m_autochooser = new SendableChooser<>();

    // Turret shooting constants for lead correction
    private static final double TURRET_EXIT_VELOCITY = 8.0; // Average exit velocity (m/s)
    private static final double TURRET_LAUNCH_ANGLE = Math.toRadians(65); // Launch angle

    public RobotContainer() {
        // Configure pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        m_swerve.runAutoBuilder();

        // Configure autos
        m_autochooser.setDefaultOption("square", Autos.simpleSquareAuto(m_swerve, m_turret));
        m_autochooser.addOption("drive under tag 28", Autos.driveUnderTagAuto(m_swerve, 28));
        m_autochooser.addOption("autoalign reef A", Autos.autoAlignReef(m_swerve, 18));
        SmartDashboard.putData(m_autochooser);

        // Default commands
        m_swerve.setDefaultCommand(swerveDefaultCommand());
        m_vision.setDefaultCommand(visionDefaultCommand());

        configureBindings();
    }

    private void configureBindings() {
        m_joystick.back().onTrue(runOnce(() -> m_swerve.getCurrentCommand().cancel()));
        m_joystick.start().whileTrue(run(() -> BlackBox.DataRecorder.recordData("heading", m_swerve.getGyroHeading())));
        m_joystick.a().onTrue(CycleCommands.createCycleCommand(m_swerve, m_turret, this::hasManualDriveInput));
    }

    private boolean hasManualDriveInput() {
        final double deadband = 0.2;
        return Math.abs(m_joystick.getLeftX()) > deadband
            || Math.abs(m_joystick.getLeftY()) > deadband
            || Math.abs(m_joystick.getRightX()) > deadband;
    }

    private ParallelCommandGroup visionDefaultCommand() {
        ParallelCommandGroup cmd = new ParallelCommandGroup();
        cmd.addCommands(run(() -> m_swerve.addVisionMeasurements(m_vision.getEstimates())));
        cmd.addRequirements(m_vision);
        return cmd;
    }

    private Command swerveDefaultCommand() {
        Command driveCommand = new RunCommand(
            () -> m_swerve.drive(new ChassisSpeeds(
                MathUtil.applyDeadband(m_joystick.getLeftX(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getLeftY(), 0.2) * k_maxlinspeedteleop,
                -MathUtil.applyDeadband(m_joystick.getRightX(), 0.2) * k_maxrotspeedteleop)),
            m_swerve);

        Command turretTrackingCommand = run(() -> {
            m_turret.aimAtFieldPoseWithLead(
                m_swerve.getPose(),
                k_basinCenter,
                m_swerve.getFieldSpeeds(),
                TURRET_EXIT_VELOCITY,
                TURRET_LAUNCH_ANGLE
            );
        });

        return new ParallelCommandGroup(driveCommand, turretTrackingCommand);
    }

    public Command getAutonomousCommand() {
        return m_autochooser.getSelected();
    }

    public void testRun() {}
}
