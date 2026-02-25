// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
 * - Base plate motor: Moves game pieces within hopper
 * - Feed ramp motor: Feeds game pieces toward shooter
 */
public class Hopper extends SubsystemBase {

  // Motors
  private final SparkFlex m_basePlateMotor;
  private final SparkFlex m_feedRampMotor;

  // Motor speeds
  private static final double BASEPLATE_FEED_SPEED = 0.6;
  private static final double BASEPLATE_REVERSE_SPEED = -0.4;
  private static final double FEEDRAMP_FEED_SPEED = 0.7;
  private static final double FEEDRAMP_REVERSE_SPEED = -0.5;

  // Current limits
  private static final int BASEPLATE_CURRENT_LIMIT = 40;
  private static final int FEEDRAMP_CURRENT_LIMIT = 40;

  public Hopper() {
    // Initialize motors
    m_basePlateMotor = new SparkFlex(CANIDs.HOPPER_BASEPLATE, MotorType.kBrushless);
    m_feedRampMotor = new SparkFlex(CANIDs.HOPPER_FEEDRAMP, MotorType.kBrushless);

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig basePlateConfig = new SparkFlexConfig();
    basePlateConfig.idleMode(IdleMode.kBrake);
    basePlateConfig.smartCurrentLimit(BASEPLATE_CURRENT_LIMIT);
    m_basePlateMotor.configure(basePlateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig feedRampConfig = new SparkFlexConfig();
    feedRampConfig.idleMode(IdleMode.kBrake);
    feedRampConfig.smartCurrentLimit(FEEDRAMP_CURRENT_LIMIT);
    m_feedRampMotor.configure(feedRampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Base Plate Control =====

  public void runBasePlate() {
    m_basePlateMotor.set(BASEPLATE_FEED_SPEED);
  }

  public void reverseBasePlate() {
    m_basePlateMotor.set(BASEPLATE_REVERSE_SPEED);
  }

  public void stopBasePlate() {
    m_basePlateMotor.set(0);
  }

  public void setBasePlateSpeed(double speed) {
    m_basePlateMotor.set(speed);
  }

  // ===== Feed Ramp Control =====

  public void runFeedRamp() {
    m_feedRampMotor.set(FEEDRAMP_FEED_SPEED);
  }

  public void reverseFeedRamp() {
    m_feedRampMotor.set(FEEDRAMP_REVERSE_SPEED);
  }

  public void stopFeedRamp() {
    m_feedRampMotor.set(0);
  }

  public void setFeedRampSpeed(double speed) {
    m_feedRampMotor.set(speed);
  }

  // ===== Combined Operations =====

  public void feedToShooter() {
    runBasePlate();
    runFeedRamp();
  }

  public void reverseAll() {
    reverseBasePlate();
    reverseFeedRamp();
  }

  public void stopAll() {
    stopBasePlate();
    stopFeedRamp();
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
    SmartDashboard.putNumber("Hopper/BasePlateOutput", m_basePlateMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hopper/FeedRampOutput", m_feedRampMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hopper/BasePlateCurrent", m_basePlateMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hopper/FeedRampCurrent", m_feedRampMotor.getOutputCurrent());
  }
}
