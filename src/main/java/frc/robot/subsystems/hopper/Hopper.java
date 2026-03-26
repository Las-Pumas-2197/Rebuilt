// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CANIDs;

/**
 * Hopper subsystem — currently houses the intake slide motor.
 * Conveyor motor is removed from robot for the time being.
 */
public class Hopper extends SubsystemBase {
  // Slide motor
  private final SparkFlex m_slideMotor;
  private final RelativeEncoder m_slideEncoder;

  private static final double SLIDE_GEAR_RATIO = 25.0;
  private static final double SLIDE_EXTEND_SPEED = -0.2;
  private static final double SLIDE_RETRACT_SPEED = 0.2;
  private static final int SLIDE_CURRENT_LIMIT = 40;

  // Robot starts closed @ 0
  private static final double SLIDE_TRAVEL_MAX_POS = -0.5;
  private static final double SLIDE_TRAVEL_MIN_POS = -0.2;

  public Hopper() {
    // Slide motor
    m_slideMotor = new SparkFlex(CANIDs.INTAKE_SLIDE, MotorType.kBrushless);
    m_slideEncoder = m_slideMotor.getEncoder();

    configureMotors();
  }

  private void configureMotors() {


    SparkFlexConfig slideConfig = new SparkFlexConfig();
    slideConfig.idleMode(IdleMode.kBrake);
    slideConfig.smartCurrentLimit(SLIDE_CURRENT_LIMIT);
    // Convert encoder from motor rotations to output shaft rotations
    slideConfig.encoder.positionConversionFactor(1.0 / SLIDE_GEAR_RATIO);
    m_slideMotor.configure(slideConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public double getSlidePosition() {
    return m_slideEncoder.getPosition();
  }

  public void zeroSlideEncoder() {
    m_slideEncoder.setPosition(0);
  }

  // ===== State =====

  private boolean m_slideExtended = false;

  // ===== Commands =====

  /** Toggles the slide between extended and retracted based on encoder position. */

  public Command autoExtendSlide() {
    return this.run(this::extendSlide)
            .until(() -> getSlidePosition() <= SLIDE_TRAVEL_MAX_POS)
            .andThen(this.runOnce(this::stopSlide));
  }

  public Command slideCommand() {
    return Commands.defer(() -> {
      m_slideExtended = !m_slideExtended;

      if (m_slideExtended) {
        return this.run(this::extendSlide)
            .until(() -> getSlidePosition() <= SLIDE_TRAVEL_MAX_POS)
            .andThen(this.runOnce(this::stopSlide));
      } else {
        return this.run(this::retractSlide)
            .until(() -> getSlidePosition() >= SLIDE_TRAVEL_MIN_POS)
            .andThen(this.runOnce(this::stopSlide));
      }
    }, Set.of(this)).withName("SlideToggle");
  }

  @Override
  public void periodic() {
    // Conveyor telemetry — motor currently removed
    // SmartDashboard.putNumber("Hopper/ConveyorOutput", m_conveyorMotor.getAppliedOutput());
    // SmartDashboard.putNumber("Hopper/ConveyorCurrent", m_conveyorMotor.getOutputCurrent());

    SmartDashboard.putNumber("Hopper/SlideOutput", m_slideMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hopper/SlideCurrent", m_slideMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hopper/SlidePosition", m_slideEncoder.getPosition());
  }
}
