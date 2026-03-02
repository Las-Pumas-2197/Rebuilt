// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CANIDs;


public class Feeder extends SubsystemBase {

  // Motors
  private final SparkFlex m_kickerMotor;
  private final SparkFlex m_feedRampMotor;

  private static final double KICKER_SPEED = 1;
  private static final double KICKER_REVERSE_SPEED = -0.5;

  private static final int KICKER_CURRENT_LIMIT = 30;
  private static final int FEEDRAMP_CURRENT_LIMIT = 40;

  private static final double FEEDRAMP_FEED_SPEED = -1;
  private static final double FEEDRAMP_REVERSE_SPEED = 1;

  public Feeder() {
    // Kicker wheel
    m_kickerMotor = new SparkFlex(CANIDs.FEEDER_KICKER, MotorType.kBrushless);

    // Feed ramp
    m_feedRampMotor = new SparkFlex(CANIDs.HOPPER_FEEDRAMP, MotorType.kBrushless);

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    SparkFlexConfig kickerConfig = new SparkFlexConfig();
    kickerConfig.idleMode(IdleMode.kBrake);
    kickerConfig.smartCurrentLimit(KICKER_CURRENT_LIMIT);
    m_kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig feedRampConfig = new SparkFlexConfig();
    feedRampConfig.idleMode(IdleMode.kBrake);
    feedRampConfig.smartCurrentLimit(FEEDRAMP_CURRENT_LIMIT);
    m_feedRampMotor.configure(feedRampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Kicker Control =====

  public void runKicker() {
    m_kickerMotor.set(KICKER_SPEED);
  }

  public void reverseKicker() {
    m_kickerMotor.set(KICKER_REVERSE_SPEED);
  }

  public void stopKicker() {
    m_kickerMotor.set(0);
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

  public void stopAllFeeder() {
    m_kickerMotor.stopMotor();
    m_feedRampMotor.stopMotor();
  }

  public void runFeeder() {
    runKicker();
    runFeedRamp();
  }
}
