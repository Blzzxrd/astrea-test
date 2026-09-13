package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Hardware and control for one drive/steer/absolute-encoder swerve module. */
public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_steerMotor;
  private final CANcoder m_canCoder;
  private final RelativeEncoder m_driveEncoder;
  private final PIDController m_steeringPid;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final String m_name;
  private final double m_canCoderOffsetRotations;
  private final double m_driveSpeedDeadbandMetersPerSecond;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Creates one module. All supplied IDs and conversion values must come from the team's hardware.
   */
  public SwerveModule(
      String name,
      int driveMotorCanId,
      int steerMotorCanId,
      int canCoderCanId,
      double canCoderOffsetRotations,
      double wheelDiameterMeters,
      double driveGearRatio,
      double steeringKp,
      double steeringKi,
      double steeringKd,
      double driveKsVolts,
      double driveKvVoltSecondsPerMeter,
      double driveSpeedDeadbandMetersPerSecond,
      boolean driveMotorInverted,
      boolean steerMotorInverted) {
    m_name = name;
    m_driveMotor = new SparkMax(driveMotorCanId, MotorType.kBrushless);
    m_steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    m_canCoder = new CANcoder(canCoderCanId);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_canCoderOffsetRotations = canCoderOffsetRotations;
    m_driveSpeedDeadbandMetersPerSecond = driveSpeedDeadbandMetersPerSecond;

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

    m_steeringPid = new PIDController(steeringKp, steeringKi, steeringKd);
    m_steeringPid.enableContinuousInput(-Math.PI, Math.PI);
    m_driveFeedforward = new SimpleMotorFeedforward(driveKsVolts, driveKvVoltSecondsPerMeter);
  }

  /** Returns this module's measured wheel speed and absolute steering angle. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }

  /** Returns this module's measured wheel distance and absolute steering angle. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getAngle());
  }

  /** Returns the offset-corrected absolute CANcoder angle, wrapped by {@link Rotation2d}. */
  public Rotation2d getAngle() {
    double rotations = m_canCoder.getAbsolutePosition().refresh().getValue().in(Rotations);
    return Rotation2d.fromRotations(rotations - m_canCoderOffsetRotations);
  }

  /**
   * Optimizes and applies a requested state. Drive uses a simple feedforward voltage; steering
   * uses the absolute CANcoder and a continuous-input WPILib PID controller.
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState optimizedState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    optimizedState.optimize(getAngle());
    m_desiredState = optimizedState;

    if (Math.abs(optimizedState.speedMetersPerSecond) < m_driveSpeedDeadbandMetersPerSecond) {
      m_driveMotor.stopMotor();
    } else {
      double driveVolts = m_driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
      m_driveMotor.setVoltage(MathUtil.clamp(driveVolts, -12.0, 12.0));
    }

    double steerOutput =
        m_steeringPid.calculate(getAngle().getRadians(), optimizedState.angle.getRadians());
    m_steerMotor.set(MathUtil.clamp(steerOutput, -1.0, 1.0));
  }

  /** Stops both NEO motors. */
  public void stop() {
    m_driveMotor.stopMotor();
    m_steerMotor.stopMotor();
  }

  /** Publishes the small set of values needed for initial module checkout. */
  public void publishTelemetry() {
    String keyPrefix = "Swerve/" + m_name + "/";
    SmartDashboard.putNumber(keyPrefix + "CANcoder Angle Degrees", getAngle().getDegrees());
    SmartDashboard.putNumber(keyPrefix + "Requested Angle Degrees", m_desiredState.angle.getDegrees());
    SmartDashboard.putNumber(keyPrefix + "Drive Velocity Meters Per Second", m_driveEncoder.getVelocity());
  }
}
