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

  // Conveyor motor — currently removed
  // private final SparkFlex m_conveyorMotor;
  // private static final double CONVEYOR_FEED_SPEED = -1;
  // private static final double CONVEYOR_REVERSE_SPEED = 1;
  // private static final int CONVEYOR_CURRENT_LIMIT = 40;

  // Slide motor (moved from Intake)
  private final SparkFlex m_slideMotor;
  private final RelativeEncoder m_slideEncoder;

  private static final double SLIDE_GEAR_RATIO = 25.0;
  private static final double SLIDE_EXTEND_SPEED = 1;
  private static final double SLIDE_RETRACT_SPEED = -1;
  private static final int SLIDE_CURRENT_LIMIT = 20;

  // TODO: measure this — start fully extended, zero encoder, retract until closed, read value
  private static final double SLIDE_TRAVEL_ROTATIONS = 0.0; // output shaft rotations (TBD)

  public Hopper() {
    // Conveyor — currently removed
    // m_conveyorMotor = new SparkFlex(CANIDs.HOPPER_CONVEYOR, MotorType.kBrushless);

    // Slide motor
    m_slideMotor = new SparkFlex(CANIDs.INTAKE_SLIDE, MotorType.kBrushless);
    m_slideEncoder = m_slideMotor.getEncoder();

    configureMotors();
  }

  private void configureMotors() {
    // Conveyor config — currently removed
    // SparkFlexConfig conveyorConfig = new SparkFlexConfig();
    // conveyorConfig.idleMode(IdleMode.kBrake);
    // conveyorConfig.smartCurrentLimit(CONVEYOR_CURRENT_LIMIT);
    // m_conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig slideConfig = new SparkFlexConfig();
    slideConfig.idleMode(IdleMode.kBrake);
    slideConfig.smartCurrentLimit(SLIDE_CURRENT_LIMIT);
    // Convert encoder from motor rotations to output shaft rotations
    slideConfig.encoder.positionConversionFactor(1.0 / SLIDE_GEAR_RATIO);
    m_slideMotor.configure(slideConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Conveyor Control (currently removed) =====

  // public void runConveyor() {
  //   m_conveyorMotor.set(CONVEYOR_FEED_SPEED);
  // }

  // public void reverseConveyor() {
  //   m_conveyorMotor.set(CONVEYOR_REVERSE_SPEED);
  // }

  // public void stopConveyor() {
  //   m_conveyorMotor.set(0);
  // }

  // public void setConveyorSpeed(double speed) {
  //   m_conveyorMotor.set(speed);
  // }

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
  public Command slideCommand() {
    return Commands.defer(() -> {
      m_slideExtended = !m_slideExtended;

      if (m_slideExtended) {
        return this.run(this::extendSlide)
            .until(() -> getSlidePosition() >= SLIDE_TRAVEL_ROTATIONS)
            .andThen(this.runOnce(this::stopSlide));
      } else {
        return this.run(this::retractSlide)
            .until(() -> getSlidePosition() <= 0)
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
