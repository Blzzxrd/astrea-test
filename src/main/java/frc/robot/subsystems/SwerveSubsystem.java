package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;
  private final SwerveDriveKinematics m_kinematics;

  // No gyro is configured in this project yet. Leave null to drive robot-relative.
  private GyroIO m_gyro;

  public SwerveSubsystem() {
    validateHardwareConfiguration();

    m_kinematics =
        new SwerveDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation, kRearLeftLocation, kRearRightLocation);
    m_frontLeft =
        createModule(
            kFrontLeftDriveMotorCanId,
            kFrontLeftSteerMotorCanId,
            kFrontLeftCanCoderCanId,
            kFrontLeftCanCoderOffsetRotations,
            kFrontLeftDriveMotorInverted,
            kFrontLeftSteerMotorInverted);
    m_frontRight =
        createModule(
            kFrontRightDriveMotorCanId,
            kFrontRightSteerMotorCanId,
            kFrontRightCanCoderCanId,
            kFrontRightCanCoderOffsetRotations,
            kFrontRightDriveMotorInverted,
            kFrontRightSteerMotorInverted);
    m_rearLeft =
        createModule(
            kRearLeftDriveMotorCanId,
            kRearLeftSteerMotorCanId,
            kRearLeftCanCoderCanId,
            kRearLeftCanCoderOffsetRotations,
            kRearLeftDriveMotorInverted,
            kRearLeftSteerMotorInverted);
    m_rearRight =
        createModule(
            kRearRightDriveMotorCanId,
            kRearRightSteerMotorCanId,
            kRearRightCanCoderCanId,
            kRearRightCanCoderOffsetRotations,
            kRearRightDriveMotorInverted,
            kRearRightSteerMotorInverted);
  }

  private SwerveModule createModule(
      int driveId,
      int steerId,
      int canCoderId,
      double canCoderOffsetRotations,
      boolean driveMotorInverted,
      boolean steerMotorInverted) {
    return new SwerveModule(
        driveId,
        steerId,
        canCoderId,
        canCoderOffsetRotations,
        kWheelDiameterMeters,
        kDriveGearRatio,
        kMaxDriveSpeedMetersPerSecond,
        kSteeringKP,
        kSteeringKI,
        kSteeringKD,
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
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
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
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  private static void validateHardwareConfiguration() {
    int[] canIds = {
      kFrontLeftDriveMotorCanId, kFrontLeftSteerMotorCanId, kFrontLeftCanCoderCanId,
      kFrontRightDriveMotorCanId, kFrontRightSteerMotorCanId, kFrontRightCanCoderCanId,
      kRearLeftDriveMotorCanId, kRearLeftSteerMotorCanId, kRearLeftCanCoderCanId,
      kRearRightDriveMotorCanId, kRearRightSteerMotorCanId, kRearRightCanCoderCanId
    };
    for (int canId : canIds) {
      if (canId < 0) {
        throw new IllegalStateException(
            "Configure all swerve CAN IDs in Constants.SwerveConstants before deploying.");
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
          "Configure the TODO swerve dimensions, ratios, speeds, and steering PID gains before deploying.");
    }
  }
}
