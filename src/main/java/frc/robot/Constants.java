// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.10;
  }

  /** Robot-level swerve dimensions and motion limits. */
  public static final class DriveConstants {
    private DriveConstants() {}

    // TODO: Measure the 2026 chassis. Do not copy these values from an older robot.
    public static final double kTrackWidthMeters = 0.0; // Left wheel center to right wheel center.
    public static final double kWheelbaseMeters = 0.0; // Front wheel center to back wheel center.
    public static final double kMaxDriveSpeedMetersPerSecond = 0.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 0.0;

    // WPILib convention: +X forward and +Y left.
    public static final Translation2d kFrontLeftLocation =
        new Translation2d(kWheelbaseMeters / 2.0, kTrackWidthMeters / 2.0);
    public static final Translation2d kFrontRightLocation =
        new Translation2d(kWheelbaseMeters / 2.0, -kTrackWidthMeters / 2.0);
    public static final Translation2d kBackLeftLocation =
        new Translation2d(-kWheelbaseMeters / 2.0, kTrackWidthMeters / 2.0);
    public static final Translation2d kBackRightLocation =
        new Translation2d(-kWheelbaseMeters / 2.0, -kTrackWidthMeters / 2.0);
  }

  /** Per-module hardware, conversion, and control constants. */
  public static final class ModuleConstants {
    private ModuleConstants() {}

    // TODO: Replace every -1 CAN ID with the ID assigned to that device in Phoenix Tuner / REV.
    public static final int kFrontLeftDriveMotorCanId = -1;
    public static final int kFrontLeftSteerMotorCanId = -1;
    public static final int kFrontLeftCanCoderCanId = -1;
    public static final int kFrontRightDriveMotorCanId = -1;
    public static final int kFrontRightSteerMotorCanId = -1;
    public static final int kFrontRightCanCoderCanId = -1;
    public static final int kBackLeftDriveMotorCanId = -1;
    public static final int kBackLeftSteerMotorCanId = -1;
    public static final int kBackLeftCanCoderCanId = -1;
    public static final int kBackRightDriveMotorCanId = -1;
    public static final int kBackRightSteerMotorCanId = -1;
    public static final int kBackRightCanCoderCanId = -1;

    // TODO: Set each value after zeroing the matching CANcoder with its wheel pointed forward.
    // Offsets are CANcoder rotations, not degrees.
    public static final double kFrontLeftCanCoderOffsetRotations = 0.0;
    public static final double kFrontRightCanCoderOffsetRotations = 0.0;
    public static final double kBackLeftCanCoderOffsetRotations = 0.0;
    public static final double kBackRightCanCoderOffsetRotations = 0.0;

    // Carried over from Astraea's 2024-Offseason and 2025-Reefscape swerve code.
    // TODO: Verify both values against the 2026 module hardware before deploying.
    public static final double kDriveGearRatio = 3.56; // Motor rotations per wheel rotation.
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kDriveMetersPerMotorRotation =
        Math.PI * kWheelDiameterMeters / kDriveGearRatio;
    public static final double kDriveMetersPerSecondPerMotorRpm =
        kDriveMetersPerMotorRotation / 60.0;

    // This direct-CANcoder steering implementation does not use the steer ratio yet. Record it
    // here for future closed-loop steering features if desired.
    public static final double kSteerGearRatio = 0.0; // Motor rotations per module rotation.

    // Carried over from prior Astraea code, where the PID angle unit was degrees.
    // The 2026 PID uses radians, so convert the retained gain to duty-cycle-per-radian.
    // TODO: Tune these gains on blocks before floor testing.
    public static final double kLegacyTurnPPerDegree = 0.004;
    public static final double kSteeringKP = kLegacyTurnPPerDegree * 180.0 / Math.PI;
    public static final double kSteeringKI = 0.0;
    public static final double kSteeringKD = 0.0;

    // Carried over from prior Astraea code. Used for simple open-loop drive feedforward.
    // TODO: Verify/tune on the 2026 robot before relying on these values.
    public static final double kDriveKSVolts = 0.25;
    public static final double kDriveKVVoltSecondsPerMeter = 6.5;

    // Keep steering active but stop the drive motor when wheel speed is effectively zero.
    public static final double kDriveSpeedDeadbandMetersPerSecond = 0.01;

    // TODO: Verify every direction on blocks. These false placeholders are not hardware facts.
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontLeftSteerMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kFrontRightSteerMotorInverted = false;
    public static final boolean kBackLeftDriveMotorInverted = false;
    public static final boolean kBackLeftSteerMotorInverted = false;
    public static final boolean kBackRightDriveMotorInverted = false;
    public static final boolean kBackRightSteerMotorInverted = false;
  }
}
