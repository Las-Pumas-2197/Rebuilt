// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

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

/**
 * Turret subsystem with flywheel shooter, rotation, and feed belt.
 * - 2 flywheel motors for shooting (left/right)
 * - 1 rotation motor for turret aiming
 * - 1 feed belt motor to feed game pieces into shooter
 */
public class Turret extends SubsystemBase {

  // Motors
  private final SparkFlex m_flywheelLeft;
  private final SparkFlex m_flywheelRight;
  private final SparkFlex m_rotationMotor;
  private final SparkFlex m_feedBeltMotor;

  // PID for turret rotation position control
  private final PIDController m_rotationPID;

  // Turret state
  private double m_targetYaw = Math.PI;      // Target yaw angle (radians)
  private double m_currentYaw = Math.PI;     // Current yaw angle (radians)

  // Turret physical limits (radians)
  private static final double MIN_YAW = Math.toRadians(135);   // 135 degrees
  private static final double MAX_YAW = Math.toRadians(-90);   // 270 degrees (-90)

  // Motor speeds
  private static final double FLYWHEEL_SHOOT_SPEED = 0.9;
  private static final double FLYWHEEL_IDLE_SPEED = 0.3;
  private static final double FEEDBELT_SPEED = 0.8;
  private static final double FEEDBELT_REVERSE_SPEED = -0.5;

  // Current limits
  private static final int FLYWHEEL_CURRENT_LIMIT = 60;
  private static final int ROTATION_CURRENT_LIMIT = 40;
  private static final int FEEDBELT_CURRENT_LIMIT = 30;

  // PID constants for rotation
  private static final double ROTATION_KP = 2.0;
  private static final double ROTATION_KI = 0.0;
  private static final double ROTATION_KD = 0.1;

  // Shooting physics constants
  private static final double LAUNCH_ANGLE      = Math.toRadians(65); // fixed launch angle (rad)
  private static final double LAUNCH_HEIGHT      = 0.3;    // turret exit height (m)
  private static final double HUB_TARGET_HEIGHT  = 1.5748; // hub opening height (m)
  private static final double MIN_EXIT_VELOCITY  = 3.0;    // m/s
  private static final double MAX_EXIT_VELOCITY  = 15.0;   // m/s
  private static final double GRAVITY            = 9.81;   // m/s²

  public Turret() {
    // Initialize motors
    m_flywheelLeft  = new SparkFlex(CANIDs.TURRET_FLYWHEEL_LEFT,  MotorType.kBrushless);
    m_flywheelRight = new SparkFlex(CANIDs.TURRET_FLYWHEEL_RIGHT, MotorType.kBrushless);
    m_rotationMotor = new SparkFlex(CANIDs.TURRET_ROTATION,       MotorType.kBrushless);
    m_feedBeltMotor = new SparkFlex(CANIDs.TURRET_FEEDBELT,       MotorType.kBrushless);

    // Initialize PID controller
    m_rotationPID = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
    m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationPID.setTolerance(Math.toRadians(2.0));

    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    // Flywheel left config
    SparkFlexConfig flywheelLeftConfig = new SparkFlexConfig();
    flywheelLeftConfig.idleMode(IdleMode.kCoast);
    flywheelLeftConfig.smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    m_flywheelLeft.configure(flywheelLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Flywheel right config (inverted to spin opposite direction)
    SparkFlexConfig flywheelRightConfig = new SparkFlexConfig();
    flywheelRightConfig.idleMode(IdleMode.kCoast);
    flywheelRightConfig.smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    flywheelRightConfig.inverted(true);
    m_flywheelRight.configure(flywheelRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Rotation motor config
    SparkFlexConfig rotationConfig = new SparkFlexConfig();
    rotationConfig.idleMode(IdleMode.kBrake);
    rotationConfig.smartCurrentLimit(ROTATION_CURRENT_LIMIT);
    m_rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Feed belt config
    SparkFlexConfig feedBeltConfig = new SparkFlexConfig();
    feedBeltConfig.idleMode(IdleMode.kBrake);
    feedBeltConfig.smartCurrentLimit(FEEDBELT_CURRENT_LIMIT);
    m_feedBeltMotor.configure(feedBeltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ===== Flywheel Control =====

  public void spinUpFlywheels() {
    m_flywheelLeft.set(FLYWHEEL_SHOOT_SPEED);
    m_flywheelRight.set(FLYWHEEL_SHOOT_SPEED);
  }

  public void idleFlywheels() {
    m_flywheelLeft.set(FLYWHEEL_IDLE_SPEED);
    m_flywheelRight.set(FLYWHEEL_IDLE_SPEED);
  }

  public void stopFlywheels() {
    m_flywheelLeft.set(0);
    m_flywheelRight.set(0);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheelLeft.set(speed);
    m_flywheelRight.set(speed);
  }

  public boolean areFlywheelsAtSpeed() {
    // TODO: Check actual RPM from encoder when tuned
    return Math.abs(m_flywheelLeft.getAppliedOutput()) > 0.85;
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

  // ===== Turret Rotation Control =====

  public void setTargetYaw(double yawRadians) {
    m_targetYaw = clampYaw(yawRadians);
  }

  public double getTargetYaw() {
    return m_targetYaw;
  }

  public double getCurrentYaw() {
    // TODO: Read from encoder when hardware is calibrated
    return m_currentYaw;
  }

  public void setCurrentYaw(double yawRadians) {
    m_currentYaw = yawRadians;
  }

  public boolean isAtTarget() {
    return m_rotationPID.atSetpoint();
  }

  public void setRotationSpeed(double speed) {
    m_rotationMotor.set(speed);
  }

  public void stopRotation() {
    m_rotationMotor.set(0);
  }

  // ===== Shooting Physics =====

  /**
   * Calculates the exit velocity required to reach a target at the given
   * horizontal distance, using projectile motion with the fixed launch angle.
   *
   * v² = g·d² / (2·cos²θ · (d·tanθ − Δh))
   *
   * @param horizontalDistance Horizontal distance to target (m)
   * @return Clamped exit velocity in m/s
   */
  public double calculateRequiredExitVelocity(double horizontalDistance) {
    double deltaH     = HUB_TARGET_HEIGHT - LAUNCH_HEIGHT;
    double tanAngle   = Math.tan(LAUNCH_ANGLE);
    double cosAngle   = Math.cos(LAUNCH_ANGLE);
    double denominator = 2.0 * cosAngle * cosAngle * (horizontalDistance * tanAngle - deltaH);

    if (denominator <= 0) {
      return MAX_EXIT_VELOCITY; // target unreachable at this angle — use max
    }

    double velocity = Math.sqrt(GRAVITY * horizontalDistance * horizontalDistance / denominator);
    return Math.max(MIN_EXIT_VELOCITY, Math.min(MAX_EXIT_VELOCITY, velocity));
  }

  /**
   * Maps a required exit velocity to a flywheel motor output (0–1).
   * Uses a linear mapping anchored at MAX_EXIT_VELOCITY → FLYWHEEL_SHOOT_SPEED.
   */
  private double velocityToMotorSpeed(double exitVelocity) {
    double speed = (exitVelocity / MAX_EXIT_VELOCITY) * FLYWHEEL_SHOOT_SPEED;
    return Math.max(FLYWHEEL_IDLE_SPEED, Math.min(FLYWHEEL_SHOOT_SPEED, speed));
  }

  // ===== Aiming Methods =====

  public double calculateAngleToFieldPose(Pose2d robotPose, Pose2d targetPose) {
    double angleToTarget = Math.atan2(
        targetPose.getY() - robotPose.getY(),
        targetPose.getX() - robotPose.getX()
    );
    double robotHeading = robotPose.getRotation().getRadians();
    return normalizeAngle(angleToTarget - robotHeading);
  }

  public void aimAtFieldPose(Pose2d robotPose, Pose2d targetPose) {
    setTargetYaw(calculateAngleToFieldPose(robotPose, targetPose));
  }

  /**
   * Calculates a lead-corrected aim angle using iterative convergence.
   *
   * @param robotPose           Current robot pose
   * @param targetPose          Field pose of the target
   * @param fieldRelativeSpeeds Field-relative chassis speeds
   * @param exitVelocity        Projectile exit velocity (m/s)
   * @param launchAngle         Launch angle above horizontal (rad)
   * @return Lead-corrected turret yaw angle relative to robot heading (rad)
   */
  public double calculateLeadCorrectedAngle(
      Pose2d robotPose,
      Pose2d targetPose,
      ChassisSpeeds fieldRelativeSpeeds,
      double exitVelocity,
      double launchAngle) {

    Translation2d robotPos  = robotPose.getTranslation();
    Translation2d targetPos = targetPose.getTranslation();
    double horizontalVelocity = exitVelocity * Math.cos(launchAngle);

    Translation2d adjustedTarget = targetPos;
    for (int i = 0; i < 3; i++) {
      double distance     = robotPos.getDistance(adjustedTarget);
      double timeOfFlight = distance / horizontalVelocity;
      double driftX = fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight;
      double driftY = fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight;
      adjustedTarget = new Translation2d(
          targetPos.getX() - driftX,
          targetPos.getY() - driftY
      );
    }

    double angleToTarget = Math.atan2(
        adjustedTarget.getY() - robotPos.getY(),
        adjustedTarget.getX() - robotPos.getX()
    );
    double robotHeading = robotPose.getRotation().getRadians();
    return normalizeAngle(angleToTarget - robotHeading);
  }

  /**
   * Aims the turret at a field pose with lead correction and dynamically sets
   * flywheel speed based on the exit velocity required to reach the target
   * from the robot's current position.
   *
   * @param robotPose           Current robot pose
   * @param targetPose          Field pose of the target (e.g. hub center)
   * @param fieldRelativeSpeeds Field-relative chassis speeds for lead correction
   */
  public void aimAtFieldPoseWithLead(
      Pose2d robotPose,
      Pose2d targetPose,
      ChassisSpeeds fieldRelativeSpeeds) {

    double distance    = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    double exitVelocity = calculateRequiredExitVelocity(distance);

    // Spin flywheels to the speed required for this distance
    setFlywheelSpeed(velocityToMotorSpeed(exitVelocity));

    // Aim with lead correction using the calculated velocity
    setTargetYaw(calculateLeadCorrectedAngle(
        robotPose, targetPose, fieldRelativeSpeeds, exitVelocity, LAUNCH_ANGLE));

    SmartDashboard.putNumber("Turret/ExitVelocity", exitVelocity);
    SmartDashboard.putNumber("Turret/DistanceToTarget", distance);
  }

  // ===== Combined Operations =====

  public void stopAll() {
    stopFlywheels();
    stopFeedBelt();
    stopRotation();
  }

  // ===== Commands =====

  public Command shootCommand() {
    return runEnd(
        () -> {
          spinUpFlywheels();
          if (areFlywheelsAtSpeed()) {
            runFeedBelt();
          }
        },
        this::stopAll
    ).withName("Shoot");
  }

  public Command spinUpCommand() {
    return runEnd(this::spinUpFlywheels, this::stopFlywheels).withName("SpinUp");
  }

  public Command idleCommand() {
    return runEnd(this::idleFlywheels, this::stopFlywheels).withName("Idle");
  }

  // ===== Utility Methods =====

  private double normalizeAngle(double angle) {
    while (angle > Math.PI)  angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }

  private double clampYaw(double yaw) {
    yaw = normalizeAngle(yaw);
    if (isYawInRange(yaw)) {
      return yaw;
    }
    double distToMin = Math.abs(normalizeAngle(yaw - MIN_YAW));
    double distToMax = Math.abs(normalizeAngle(yaw - MAX_YAW));
    return distToMin < distToMax ? MIN_YAW : MAX_YAW;
  }

  private boolean isYawInRange(double yaw) {
    return yaw >= MIN_YAW || yaw <= MAX_YAW;
  }

  public boolean isYawReachable(double yaw) {
    return isYawInRange(normalizeAngle(yaw));
  }

  public boolean isAtLimit() {
    final double tolerance = Math.toRadians(2.0);
    return Math.abs(m_currentYaw - MIN_YAW) < tolerance
        || Math.abs(m_currentYaw - MAX_YAW) < tolerance;
  }

  @Override
  public void periodic() {
    // Run rotation PID control
    double rotationOutput = m_rotationPID.calculate(m_currentYaw, m_targetYaw);
    m_rotationMotor.set(rotationOutput);

    // Telemetry
    SmartDashboard.putNumber("Turret/TargetYaw",           Math.toDegrees(m_targetYaw));
    SmartDashboard.putNumber("Turret/CurrentYaw",          Math.toDegrees(m_currentYaw));
    SmartDashboard.putBoolean("Turret/AtTarget",           isAtTarget());
    SmartDashboard.putBoolean("Turret/AtLimit",            isAtLimit());
    SmartDashboard.putNumber("Turret/FlywheelLeftOutput",  m_flywheelLeft.getAppliedOutput());
    SmartDashboard.putNumber("Turret/FlywheelRightOutput", m_flywheelRight.getAppliedOutput());
    SmartDashboard.putNumber("Turret/FeedBeltOutput",      m_feedBeltMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Turret/FlywheelsAtSpeed",   areFlywheelsAtSpeed());
  }
}
