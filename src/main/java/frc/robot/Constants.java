// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

  /** Constants that must be measured or verified on the actual robot before it is deployed. */
  public static final class SwerveConstants {
    private SwerveConstants() {}

    // TODO: Replace every -1 CAN ID with the ID assigned to that device in Phoenix Tuner / REV.
    public static final int kFrontLeftDriveMotorCanId = -1;
    public static final int kFrontLeftSteerMotorCanId = -1;
    public static final int kFrontLeftCanCoderCanId = -1;
    public static final int kFrontRightDriveMotorCanId = -1;
    public static final int kFrontRightSteerMotorCanId = -1;
    public static final int kFrontRightCanCoderCanId = -1;
    public static final int kRearLeftDriveMotorCanId = -1;
    public static final int kRearLeftSteerMotorCanId = -1;
    public static final int kRearLeftCanCoderCanId = -1;
    public static final int kRearRightDriveMotorCanId = -1;
    public static final int kRearRightSteerMotorCanId = -1;
    public static final int kRearRightCanCoderCanId = -1;

    // TODO: Set each value after zeroing the matching CANcoder with its wheel pointed forward.
    // Offsets are CANcoder rotations, not degrees.
    public static final double kFrontLeftCanCoderOffsetRotations = 0.0;
    public static final double kFrontRightCanCoderOffsetRotations = 0.0;
    public static final double kRearLeftCanCoderOffsetRotations = 0.0;
    public static final double kRearRightCanCoderOffsetRotations = 0.0;

    // TODO: Measure and enter this module's actual hardware values before deploying.
    public static final double kWheelDiameterMeters = 0.0;
    public static final double kDriveGearRatio = 0.0; // Motor rotations per wheel rotation.
    // This direct-CANcoder steering implementation does not use the steer ratio yet. Record it
    // here for future closed-loop steering features if desired.
    public static final double kSteerGearRatio = 0.0; // Motor rotations per module rotation.
    public static final double kTrackWidthMeters = 0.0; // Left wheel center to right wheel center.
    public static final double kWheelbaseMeters = 0.0; // Front wheel center to rear wheel center.
    public static final double kMaxDriveSpeedMetersPerSecond = 0.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = 0.0;

    // TODO: Tune on blocks, starting with kSteeringKP. The PID output is motor duty cycle.
    public static final double kSteeringKP = 0.0;
    public static final double kSteeringKI = 0.0;
    public static final double kSteeringKD = 0.0;

    // TODO: Verify every direction on blocks. These false placeholders are not hardware facts.
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontLeftSteerMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kFrontRightSteerMotorInverted = false;
    public static final boolean kRearLeftDriveMotorInverted = false;
    public static final boolean kRearLeftSteerMotorInverted = false;
    public static final boolean kRearRightDriveMotorInverted = false;
    public static final boolean kRearRightSteerMotorInverted = false;

    // WPILib convention: +X forward and +Y left.
    public static final Translation2d kFrontLeftLocation =
        new Translation2d(kWheelbaseMeters / 2.0, kTrackWidthMeters / 2.0);
    public static final Translation2d kFrontRightLocation =
        new Translation2d(kWheelbaseMeters / 2.0, -kTrackWidthMeters / 2.0);
    public static final Translation2d kRearLeftLocation =
        new Translation2d(-kWheelbaseMeters / 2.0, kTrackWidthMeters / 2.0);
    public static final Translation2d kRearRightLocation =
        new Translation2d(-kWheelbaseMeters / 2.0, -kTrackWidthMeters / 2.0);
  }
}
