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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CANIDs;


public class Feeder extends SubsystemBase {

  // Motors
  private final SparkFlex m_feedBeltMotor;
  private final SparkFlex m_feedRampMotor;

  private static final double FEEDBELT_SPEED = 1;
  private static final double FEEDBELT_REVERSE_SPEED = -0.5;

  private static final int FEEDBELT_CURRENT_LIMIT = 30;
  private static final int FEEDRAMP_CURRENT_LIMIT = 40;

  private static final double FEEDRAMP_FEED_SPEED = -1;
  private static final double FEEDRAMP_REVERSE_SPEED = 1;

  public Feeder() {
    // Kicker wheel
    m_feedBeltMotor = new SparkFlex(CANIDs.TURRET_FEEDBELT, MotorType.kBrushless);
   
   // Feed Belt
    m_feedRampMotor = new SparkFlex(CANIDs.HOPPER_FEEDRAMP, MotorType.kBrushless);

    // Configure motors
    configureMotors();
  }
  
  private void configureMotors() {
     // Feed belt config
    SparkFlexConfig feedBeltConfig = new SparkFlexConfig();
    feedBeltConfig.idleMode(IdleMode.kBrake);
    feedBeltConfig.smartCurrentLimit(FEEDBELT_CURRENT_LIMIT);
    m_feedBeltMotor.configure(feedBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig feedRampConfig = new SparkFlexConfig();
    feedRampConfig.idleMode(IdleMode.kBrake);
    feedRampConfig.smartCurrentLimit(FEEDRAMP_CURRENT_LIMIT);
    m_feedRampMotor.configure(feedRampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Feed Belt Control =====

  public void runFeedBelt() {
    m_feedBeltMotor.set(FEEDBELT_SPEED);
  }

  public void reverseFeedBelt() {
    m_feedBeltMotor.set(FEEDBELT_REVERSE_SPEED);
  }

  public void stopFeedBelt() {
    m_feedBeltMotor.set(0);
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

  //combined functions
  public void stopAllFeeder() {
    m_feedBeltMotor.stopMotor();
    m_feedRampMotor.stopMotor();
  }

  public void runFeeder() {
    this.runFeedBelt();
    this.runFeedRamp();
  }
}