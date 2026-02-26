// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CANIDs;

/**
 * Intake subsystem with slide and roller motors.
 * - Slide motor: Extends/retracts the intake mechanism
 * - Roller motor: Spins to intake game pieces
 */
public class Intake extends SubsystemBase {

  // Motors
  private final SparkFlex m_slideMotor;
  private final SparkFlex m_rollerMotor;

  // Motor speeds
  private static final double SLIDE_EXTEND_SPEED = 0.5;
  private static final double SLIDE_RETRACT_SPEED = -0.5;
  private static final double ROLLER_INTAKE_SPEED = 0.8;
  private static final double ROLLER_EJECT_SPEED = -0.6;

  // Current limits
  private static final int SLIDE_CURRENT_LIMIT = 15;
  private static final int ROLLER_CURRENT_LIMIT = 40;

  public Intake() {
    // Initialize motors
    m_slideMotor = new SparkFlex(CANIDs.INTAKE_SLIDE, MotorType.kBrushless);
    m_rollerMotor = new SparkFlex(CANIDs.INTAKE_ROLLER, MotorType.kBrushless);

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig slideConfig = new SparkFlexConfig();
    slideConfig.idleMode(IdleMode.kBrake);
    slideConfig.smartCurrentLimit(SLIDE_CURRENT_LIMIT);
    m_slideMotor.configure(slideConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Slide Control =====

  public void extendSlide() {
    m_slideMotor.set(SLIDE_EXTEND_SPEED);
  }

  public void retractSlide() {
    m_slideMotor.set(SLIDE_RETRACT_SPEED);
  }

  public void stopSlide() {
    m_slideMotor.set(0);
  }

  public void setSlideSpeed(double speed) {
    m_slideMotor.set(speed);
  }

  // ===== Roller Control =====

  public void runIntake() {
    m_rollerMotor.set(ROLLER_INTAKE_SPEED);
  }

  public void runEject() {
    m_rollerMotor.set(ROLLER_EJECT_SPEED);
  }

  public void stopRoller() {
    m_rollerMotor.set(0);
  }

  public void setRollerSpeed(double speed) {
    m_rollerMotor.set(speed);
  }

  // ===== Combined Operations =====

  public void stopAll() {
    stopSlide();
    stopRoller();
  }

  // ===== State =====

  private boolean m_slideExtended = false;

  // ===== Commands =====

  /** Toggles the slide between extended and retracted, running the motor for 1.5s then stopping. */
  public Command slideCommand() {
    return Commands.defer(() -> {
      m_slideExtended = !m_slideExtended;
      Runnable motorAction = m_slideExtended ? this::extendSlide : this::retractSlide;
      
      return this.run(motorAction).withTimeout(1.5).andThen(this.runOnce(this::stopSlide));
    }, Set.of(this)).withName("SlideToggle");
  }

  /** Runs the roller while held, stops on release. */
  public Command rollerCommand() {
    return runEnd(this::runIntake, this::stopRoller).withName("Roller");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/SlideOutput", m_slideMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/RollerOutput", m_rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/SlideCurrent", m_slideMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/RollerCurrent", m_rollerMotor.getOutputCurrent());
  }
}
