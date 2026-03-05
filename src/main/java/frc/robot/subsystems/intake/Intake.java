// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CANIDs;

/**
 * Intake subsystem with roller motor.
 * Slide motor has been moved to the Hopper subsystem.
 */
public class Intake extends SubsystemBase {

  // Motors
  private final SparkFlex m_rollerMotor;

  // Motor speeds
  private static final double ROLLER_INTAKE_SPEED = 1;
  private static final double ROLLER_EJECT_SPEED = -1;

  // Current limits
  private static final int ROLLER_CURRENT_LIMIT = 40;

  public Intake() {
    m_rollerMotor = new SparkFlex(CANIDs.INTAKE_ROLLER, MotorType.kBrushless);
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
    m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  // ===== Commands =====

  /** Runs the roller while held, stops on release. */
  public Command rollerCommand() {
    return runEnd(this::runIntake, this::stopRoller).withName("Roller");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/RollerOutput", m_rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/RollerCurrent", m_rollerMotor.getOutputCurrent());
  }
}
