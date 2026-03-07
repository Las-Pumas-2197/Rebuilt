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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  // PID for turret rotation position control
  private final TrapezoidProfile.Constraints m_rotationConstraints;
  private final ProfiledPIDController m_rotationPID;
  private final SimpleMotorFeedforward m_rotationFF;

  // Turret state — encoder starts at 0 on boot (turret at home position)
  private double m_targetYaw = 0.0;
  private double m_currentYaw = 0.0;

  // Rotation limits: ±90° from home position (0 = forward)
  private static final double MIN_YAW = -Math.PI / 2;  // -90°
  private static final double MAX_YAW =  Math.PI / 2;  //  +90°

  // Motor speeds
  private static final double FLYWHEEL_SHOOT_SPEED = 0.5;
  private static final double FLYWHEEL_IDLE_SPEED = 0.3;

  // Current limits
  private static final int FLYWHEEL_CURRENT_LIMIT = 60;
  private static final int ROTATION_CURRENT_LIMIT = 40;

  // Gear ratio: 10 motor rotations per 1 turret rotation
  private static final double TURRET_GEAR_RATIO = 10.0;

  // PID constants for rotation
  private static final double ROTATION_KP = 3;
  private static final double ROTATION_KI = 0.0;
  private static final double ROTATION_KD = 0.1;
  private static final double ROTATION_KS = 0.2;
  private static final double ROTATION_KV = 0.1689;

  // Flywheel speed calibration — two tested (distance, motor speed) pairs
  private static final double CALIB_DIST_CLOSE  = 2.0;   // close test distance (m)
  private static final double CALIB_SPEED_CLOSE = 0.35;  // motor speed that scored at close distance
  private static final double CALIB_DIST_FAR    = 5.0;   // far test distance (m)
  private static final double CALIB_SPEED_FAR   = 0.7;   // motor speed that scored at far distance
  private static final double FLYWHEEL_MIN_SPEED = 0.3;  // minimum flywheel output
  private static final double FLYWHEEL_MAX_SPEED = 0.8;  // maximum flywheel output

  public Turret() {
    // Initialize motors
    m_flywheelLeft  = new SparkFlex(CANIDs.TURRET_FLYWHEEL_LEFT,  MotorType.kBrushless);
    m_flywheelRight = new SparkFlex(CANIDs.TURRET_FLYWHEEL_RIGHT, MotorType.kBrushless);
    m_rotationMotor = new SparkFlex(CANIDs.TURRET_ROTATION,       MotorType.kBrushless);

    // Initialize PID controller
    m_rotationConstraints = new TrapezoidProfile.Constraints(11.306*Math.PI, 10*Math.PI);
    m_rotationPID = new ProfiledPIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD, m_rotationConstraints);
    m_rotationFF = new SimpleMotorFeedforward(ROTATION_KS, ROTATION_KV);
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
    rotationConfig.inverted(true);
    
    // Convert encoder position to turret radians directly
    rotationConfig.encoder.positionConversionFactor((2 * Math.PI) / TURRET_GEAR_RATIO);
    m_rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    // check actual RPM from encoder to tune
    return Math.abs(m_flywheelLeft.getAppliedOutput()) > 0.85;
  }

  // ===== Turret Rotation Control =====

  public void setTargetYaw(double yawRadians) {
    m_targetYaw = clampYaw(yawRadians);
  }

  public double getTargetYaw() {
    return m_targetYaw;
  }

  public double getCurrentYaw() {
    return m_currentYaw;
  }

  /**
   * Zeros the rotation encoder at the turret's current physical position.
   * Call this once at match start with the turret at the known home position
   * (turret pointing forward relative to robot = 0 radians).
   */
  public void zeroTurret() {
    m_rotationMotor.getEncoder().setPosition(0);
    m_currentYaw = 0;
    m_targetYaw = 0;
  }

  public boolean isAtTarget() {
    return m_rotationPID.atSetpoint();
  }

  public void setRotationSpeed(double speed) {
    m_rotationMotor.set(speed);
  }

  /**
   * Increments the target yaw by the given delta (radians).
   * Clamped to [MIN_YAW, MAX_YAW] — call this for manual joystick control
   * so limits are enforced and PID stays in control of the motor.
   */
  public void nudgeTargetYaw(double deltaRadians) {
    setTargetYaw(m_targetYaw + deltaRadians);
  }

  public void stopRotation() {
    m_rotationMotor.set(0);
  }

  public void stopAllShooter() {
    m_flywheelLeft.stopMotor();
    m_flywheelRight.stopMotor();
  }

  // ===== Flywheel Speed Interpolation =====

  /**
   * Linearly interpolates flywheel motor speed from two calibration points.
   * Extrapolates beyond the calibrated range, clamped to [MIN, MAX].
   *
   * @param distance Horizontal distance to target (m)
   * @return Motor speed (0–1)
   */
  public double interpolateFlywheelSpeed(double distance) {
    double speed = CALIB_SPEED_CLOSE
        + (distance - CALIB_DIST_CLOSE)
        * (CALIB_SPEED_FAR - CALIB_SPEED_CLOSE)
        / (CALIB_DIST_FAR - CALIB_DIST_CLOSE);
    return Math.max(FLYWHEEL_MIN_SPEED, Math.min(FLYWHEEL_MAX_SPEED, speed));
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

    double distance     = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    double motorSpeed   = interpolateFlywheelSpeed(distance);

    // Spin flywheels to the interpolated speed for this distance
    setFlywheelSpeed(motorSpeed);

    // Aim with lead correction — use motor speed as a rough proxy for horizontal velocity
    double horizontalVelocity = motorSpeed * 10.0; // rough m/s estimate, tune if lead matters
    setTargetYaw(calculateLeadCorrectedAngle(
        robotPose, targetPose, fieldRelativeSpeeds, horizontalVelocity, 0));

    SmartDashboard.putNumber("Turret/FlywheelInterpolated", motorSpeed);
    SmartDashboard.putNumber("Turret/DistanceToTarget", distance);
  }

  public void turretCL(double vel, double pos) {
    m_rotationPID.setGoal(clampYaw(pos)); 
    double turretPIDout = (m_rotationPID.calculate(m_currentYaw) / (2*Math.PI)) * 12;
    double turretFFout = m_rotationFF.calculate(m_rotationPID.getSetpoint().velocity);
    SmartDashboard.putNumber("turret FF out", turretFFout);
    SmartDashboard.putNumber("turret PID out", turretPIDout);
    SmartDashboard.putNumber("turret vel", vel);
    m_rotationMotor.setVoltage(turretFFout + turretPIDout);

    m_flywheelLeft.setVoltage(vel * 12);
    m_flywheelRight.setVoltage(vel * 12);
  }

  // ===== Combined Operations =====

  public void stopAll() {
    stopFlywheels();
    stopRotation();
  }

  // ===== Commands =====

  public Command shootCommand() {
    return runEnd(
        () -> {
          spinUpFlywheels();
          if (areFlywheelsAtSpeed()) {
            // runKicker();
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
    return Math.max(MIN_YAW, Math.min(MAX_YAW, yaw));
  }

  private boolean isYawInRange(double yaw) {
    return yaw >= MIN_YAW && yaw <= MAX_YAW;
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
    // Read current turret angle from encoder (radians, zeroed at home position)
    m_currentYaw = m_rotationMotor.getEncoder().getPosition();
    

    // Run rotation PID control
    // double rotationOutput = m_rotationPID.calculate(m_currentYaw, m_targetYaw);
    // m_rotationMotor.set(rotationOutput);

    // Telemetry
    SmartDashboard.putNumber("Turret/TargetYaw",           Math.toDegrees(m_targetYaw));
    SmartDashboard.putNumber("Turret/CurrentYaw",          Math.toDegrees(m_currentYaw));
    SmartDashboard.putBoolean("Turret/AtTarget",           isAtTarget());
    SmartDashboard.putBoolean("Turret/AtLimit",            isAtLimit());
    SmartDashboard.putNumber("Turret/FlywheelLeftOutput",  m_flywheelLeft.getAppliedOutput());
    SmartDashboard.putNumber("Turret/FlywheelRightOutput", m_flywheelRight.getAppliedOutput());
    // SmartDashboard.putNumber("Turret/FeedBeltOutput",      m_feedBeltMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Turret/FlywheelsAtSpeed",   areFlywheelsAtSpeed());
  }
}
