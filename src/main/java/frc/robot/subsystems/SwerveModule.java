package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Hardware and control for one drive/steer/absolute-encoder swerve module. */
public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_steerMotor;
  private final CANcoder m_canCoder;
  private final RelativeEncoder m_driveEncoder;
  private final PIDController m_steeringPid;
  private final double m_maxDriveSpeedMetersPerSecond;

  /**
   * Creates one module. All supplied IDs and conversion values must come from the team's hardware.
   */
  public SwerveModule(
      int driveMotorCanId,
      int steerMotorCanId,
      int canCoderCanId,
      double canCoderOffsetRotations,
      double wheelDiameterMeters,
      double driveGearRatio,
      double maxDriveSpeedMetersPerSecond,
      double steeringKp,
      double steeringKi,
      double steeringKd,
      boolean driveMotorInverted,
      boolean steerMotorInverted) {
    m_driveMotor = new SparkMax(driveMotorCanId, MotorType.kBrushless);
    m_steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    m_canCoder = new CANcoder(canCoderCanId);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_maxDriveSpeedMetersPerSecond = maxDriveSpeedMetersPerSecond;

    // Convert the NEO's motor rotations/RPM to wheel meters/meters per second.
    double driveMetersPerMotorRotation = Math.PI * wheelDiameterMeters / driveGearRatio;
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveMotorInverted)
        .idleMode(IdleMode.kBrake);
    driveConfig.encoder
        .positionConversionFactor(driveMetersPerMotorRotation)
        .velocityConversionFactor(driveMetersPerMotorRotation / 60.0);
    m_driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig.inverted(steerMotorInverted).idleMode(IdleMode.kBrake);
    m_steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // The offset makes a wheel pointed forward report an angle of zero.
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = canCoderOffsetRotations;
    m_canCoder.getConfigurator().apply(canCoderConfig);

    m_steeringPid = new PIDController(steeringKp, steeringKi, steeringKd);
    m_steeringPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Returns this module's measured wheel speed and absolute steering angle. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }

  /** Returns this module's measured wheel distance and absolute steering angle. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getAngle());
  }

  /** Returns the offset-corrected absolute CANcoder angle. */
  public Rotation2d getAngle() {
    double rotations = m_canCoder.getAbsolutePosition().refresh().getValue().in(Rotations);
    return Rotation2d.fromRotations(rotations);
  }

  /**
   * Optimizes and applies a requested state. Drive uses open-loop duty cycle; steering uses the
   * absolute CANcoder and a continuous-input WPILib PID controller.
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    double driveOutput = optimizedState.speedMetersPerSecond / m_maxDriveSpeedMetersPerSecond;
    m_driveMotor.set(MathUtil.clamp(driveOutput, -1.0, 1.0));

    double steerOutput =
        m_steeringPid.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());
    m_steerMotor.set(MathUtil.clamp(steerOutput, -1.0, 1.0));
  }

  /** Stops both NEO motors. */
  public void stop() {
    m_driveMotor.stopMotor();
    m_steerMotor.stopMotor();
  }
}
