package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ModuleConstants.*;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Minimal four-module swerve drivetrain. A real gyro can be attached through {@link GyroIO}. */
public class SwerveSubsystem extends SubsystemBase {
  /** Small abstraction so the eventual gyro vendor library is isolated from drivetrain code. */
  @FunctionalInterface
  public interface GyroIO {
    Rotation2d getRotation();
  }

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private final SwerveDriveKinematics m_kinematics;

  // The older Astraea robots used navX, but this project has no navX dependency or confirmed gyro.
  // Leave null to drive robot-relative until the 2026 gyro hardware is verified.
  private GyroIO m_gyro;

  public SwerveSubsystem() {
    validateHardwareConfiguration();

    m_kinematics =
        new SwerveDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
    m_frontLeft =
        createModule(
            "Front Left",
            kFrontLeftDriveMotorCanId,
            kFrontLeftSteerMotorCanId,
            kFrontLeftCanCoderCanId,
            kFrontLeftCanCoderOffsetRotations,
            kFrontLeftDriveMotorInverted,
            kFrontLeftSteerMotorInverted);
    m_frontRight =
        createModule(
            "Front Right",
            kFrontRightDriveMotorCanId,
            kFrontRightSteerMotorCanId,
            kFrontRightCanCoderCanId,
            kFrontRightCanCoderOffsetRotations,
            kFrontRightDriveMotorInverted,
            kFrontRightSteerMotorInverted);
    m_backLeft =
        createModule(
            "Back Left",
            kBackLeftDriveMotorCanId,
            kBackLeftSteerMotorCanId,
            kBackLeftCanCoderCanId,
            kBackLeftCanCoderOffsetRotations,
            kBackLeftDriveMotorInverted,
            kBackLeftSteerMotorInverted);
    m_backRight =
        createModule(
            "Back Right",
            kBackRightDriveMotorCanId,
            kBackRightSteerMotorCanId,
            kBackRightCanCoderCanId,
            kBackRightCanCoderOffsetRotations,
            kBackRightDriveMotorInverted,
            kBackRightSteerMotorInverted);
  }

  private SwerveModule createModule(
      String name,
      int driveId,
      int steerId,
      int canCoderId,
      double canCoderOffsetRotations,
      boolean driveMotorInverted,
      boolean steerMotorInverted) {
    return new SwerveModule(
        name,
        driveId,
        steerId,
        canCoderId,
        canCoderOffsetRotations,
        kWheelDiameterMeters,
        kDriveGearRatio,
        kSteeringKP,
        kSteeringKI,
        kSteeringKD,
        kDriveKSVolts,
        kDriveKVVoltSecondsPerMeter,
        kDriveSpeedDeadbandMetersPerSecond,
        driveMotorInverted,
        steerMotorInverted);
  }

  /**
   * Drives with +X forward, +Y left, and positive rotation counterclockwise. Without a configured
   * gyro, requesting field-relative driving safely falls back to robot-relative driving.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative && m_gyro != null) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxDriveSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  /** Adds the eventual gyro implementation without coupling this subsystem to a gyro vendor API. */
  public void setGyro(GyroIO gyro) {
    m_gyro = gyro;
  }

  /** Returns the gyro heading, or zero while the robot is intentionally gyro-less. */
  public Rotation2d getHeading() {
    return m_gyro == null ? new Rotation2d() : m_gyro.getRotation();
  }

  /** Stops all four modules. */
  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  @Override
  public void periodic() {
    m_frontLeft.publishTelemetry();
    m_frontRight.publishTelemetry();
    m_backLeft.publishTelemetry();
    m_backRight.publishTelemetry();
    if (m_gyro != null) {
      SmartDashboard.putNumber("Swerve/Gyro Heading Degrees", getHeading().getDegrees());
    }
  }

  private static void validateHardwareConfiguration() {
    int[] canIds = {
      kFrontLeftDriveMotorCanId, kFrontLeftSteerMotorCanId, kFrontLeftCanCoderCanId,
      kFrontRightDriveMotorCanId, kFrontRightSteerMotorCanId, kFrontRightCanCoderCanId,
      kBackLeftDriveMotorCanId, kBackLeftSteerMotorCanId, kBackLeftCanCoderCanId,
      kBackRightDriveMotorCanId, kBackRightSteerMotorCanId, kBackRightCanCoderCanId
    };
    Set<Integer> assignedIds = new HashSet<>();
    for (int canId : canIds) {
      if (canId < 0) {
        throw new IllegalStateException(
            "Configure all swerve CAN IDs in Constants.ModuleConstants before deploying.");
      }
      if (!assignedIds.add(canId)) {
        throw new IllegalStateException("Every swerve device must have a unique CAN ID.");
      }
    }
    if (kWheelDiameterMeters <= 0.0
        || kDriveGearRatio <= 0.0
        || kTrackWidthMeters <= 0.0
        || kWheelbaseMeters <= 0.0
        || kMaxDriveSpeedMetersPerSecond <= 0.0
        || kMaxAngularSpeedRadiansPerSecond <= 0.0
        || kSteeringKP <= 0.0) {
      throw new IllegalStateException(
          "Configure the TODO swerve dimensions and speed limits before deploying.");
    }
  }
}
