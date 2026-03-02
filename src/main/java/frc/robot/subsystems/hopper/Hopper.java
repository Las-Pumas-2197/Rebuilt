// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

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
 * Hopper subsystem for game piece storage and feeding.
 * - Conveyor motor: Moves game pieces within hopper
 * - Feed ramp motor: Feeds game pieces toward shooter
 */
public class Hopper extends SubsystemBase {

  // Motors
  private final SparkFlex m_conveyorMotor;

  // Motor speeds
  private static final double CONVEYOR_FEED_SPEED = 0.1;
  private static final double CONVEYOR_REVERSE_SPEED = -0.1;

  // Current limits
  private static final int CONVEYOR_CURRENT_LIMIT = 40;

  public Hopper() {
    // Initialize motors
    m_conveyorMotor = new SparkFlex(CANIDs.HOPPER_CONVEYOR, MotorType.kBrushless);

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig conveyorConfig = new SparkFlexConfig();
    conveyorConfig.idleMode(IdleMode.kBrake);
    conveyorConfig.smartCurrentLimit(CONVEYOR_CURRENT_LIMIT);
    m_conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Conveyor Control =====

  public void runConveyor() {
    m_conveyorMotor.set(CONVEYOR_FEED_SPEED);
  }

  public void reverseConveyor() {
    m_conveyorMotor.set(CONVEYOR_REVERSE_SPEED);
  }

  public void stopConveyor() {
    m_conveyorMotor.set(0);
  }

  public void setConveyorSpeed(double speed) {
    m_conveyorMotor.set(speed);
  }

  // ===== Combined Operations =====

  public void feedToShooter() {
    runConveyor();
    // runFeedRamp();
  }

  public void reverseAll() {
    reverseConveyor();
    // reverseFeedRamp();
  }

  public void stopAll() {
    stopConveyor();
    // stopFeedRamp();
  }

  // ===== Commands =====

  public Command feedCommand() {
    return runEnd(this::feedToShooter, this::stopAll).withName("Feed");
  }

  public Command reverseCommand() {
    return runEnd(this::reverseAll, this::stopAll).withName("Reverse");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper/ConveyorOutput", m_conveyorMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Hopper/FeedRampOutput", m_feedRampMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hopper/ConveyorCurrent", m_conveyorMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Hopper/FeedRampCurrent", m_feedRampMotor.getOutputCurrent());
  }
}
