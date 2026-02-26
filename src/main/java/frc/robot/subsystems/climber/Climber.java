// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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
 * Climber subsystem with a single motor for extending/retracting a telescoping hook.
 */
public class Climber extends SubsystemBase {

  // Motor
  private final SparkFlex m_hookMotor;

  // Motor speeds
  private static final double EXTEND_SPEED = 0.8;
  private static final double RETRACT_SPEED = -0.8;

  // Current limit
  private static final int HOOK_CURRENT_LIMIT = 60;

  public Climber() {
    m_hookMotor = new SparkFlex(CANIDs.CLIMBER_HOOK, MotorType.kBrushless);
    configureMotor();
  }

  private void configureMotor() {
    SparkFlexConfig hookConfig = new SparkFlexConfig();
    hookConfig.idleMode(IdleMode.kBrake);
    hookConfig.smartCurrentLimit(HOOK_CURRENT_LIMIT);
    m_hookMotor.configure(hookConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Hook Control =====

  public void extend() {
    m_hookMotor.set(EXTEND_SPEED);
  }

  public void retract() {
    m_hookMotor.set(RETRACT_SPEED);
  }

  public void stop() {
    m_hookMotor.set(0);
  }

  public void setSpeed(double speed) {
    m_hookMotor.set(speed);
  }

  // ===== State =====

  private boolean m_extended = false;

  // ===== Commands =====

  /** Toggles the climber between extended and retracted, running the motor for 1.5s then stopping. */
  public Command climbCommand() {
    return Commands.defer(() -> {
      m_extended = !m_extended;
      Runnable motorAction = m_extended ? this::extend : this::retract;
      
      return this.run(motorAction).withTimeout(1.5).andThen(this.runOnce(this::stop));
    }, Set.of(this)).withName("ClimbToggle");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/HookOutput", m_hookMotor.getAppliedOutput());
    SmartDashboard.putNumber("Climber/HookCurrent", m_hookMotor.getOutputCurrent());
  }
}
