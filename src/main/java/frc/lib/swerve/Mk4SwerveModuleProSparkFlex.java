package frc.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TunablePID;
import frc.lib.pid.TunableSparkPIDController;
import frc.lib.subsystem.AdvancedSubsystem;
import com.revrobotics.*;

/**
 * Implementation for an SDS Mk4 swerve module using RevNeo Vortex with
 * SparkFlex controller
 */
public class Mk4SwerveModuleProSparkFlex extends AdvancedSubsystem {
  public enum ModuleCode {
    FL,
    FR,
    BL,
    BR
  }

  // // Volts to meters/sec
  // private static final double DRIVE_KV = 3.4;
  // // Volts to meters/sec^2
  // private static final double DRIVE_KA = 0.27;

  // // Volts to deg/sec
  // private static final double ROTATION_KV = 12.0 / 900;
  // // Volts to deg/sec^2
  // private static final double ROTATION_KA = 0.00006;

  private static final double DRIVE_GEARING = 1.0 / 6.12;
  private static final double DRIVE_METERS_PER_ROTATION = DRIVE_GEARING * Math.PI * Units.inchesToMeters(4.0);
  private static final double ROTATION_DEGREES_PER_ROTATION = (1.0 / (150 / 7)) * 360.0;

  // M/s - Tune (Apply full output and measure max vel. Adjust KV/KA for sim if
  // needed)
  public static final double DRIVE_MAX_VEL = 4.65;

  private static final double DRIVE_KP = 0.0000004;
  private static final double DRIVE_KI = 0.000001;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_I_ZONE = 1000;
  private static final double DRIVE_FEED_FORWARD = 0.000149;

  private static final double ROTATION_KP = 0.00002;
  private static final double ROTATION_KI = 0.0;
  private static final double ROTATION_KD = 0.0;
  private static final double ROTATION_I_ZONE = 0;
  private static final double ROTATION_FEED_FORWARD = 0.000149;
  private static final double ROTATION_MAX_VELOCITY = 6700;
  private static final double ROTATION_MAX_ACCELERATION = 10000;
  private static final double ROTATION_ERROR = 0.05;

  // private static final double ROTATION_POSITION_KP = 0.9;
  // private static final double ROTATION_POSITION_KI = 0.0;
  // private static final double ROTATION_POSITION_KD = 0.0;
  // private static final double ROTATION_POSITION_I_ZONE = 0;
  // private static final double ROTATION_POSITION_FEED_FORWARD = 0.0;
  // private static final double ROTATION_POSITION_MAX_VELOCITY = 0;
  // private static final double ROTATION_POSITION_MAX_ACCELERATION = 0;
  // private static final double ROTATION_POSITION_ERROR = 0.5;

  public final ModuleCode moduleCode;
  // private final LinearSystemSim<N1, N1, N1> driveSim;
  // private final LinearSystemSim<N2, N1, N1> rotationSim;

  private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
  private final SparkFlex driveMotor;
  // private final REVPhysicsSim driveSimState;

  private final SparkMaxConfig rotationMotorConfig = new SparkMaxConfig();
  private final SparkFlex rotationMotor;
  // private final REVPhysicsSim rotationSimState;

  private final CANcoder rotationEncoder;
  private final CANcoderConfiguration rotationEncoderConfig;
  // private final CANcoderSimState rotationEncoderSimState;
  private final StatusSignal<Angle> rotationAbsoluteSignal;
  private final StatusSignal<AngularVelocity> rotationAbsoluteVelSignal;

  private SwerveModuleState targetState = new SwerveModuleState();

  private final TunablePID steerTunable;
  private final TunablePID driveTunable;

  /**
   * Create a Mk4 swerve module
   *
   * @param moduleCode         The code representing this module
   * @param driveMotorCanID    The CAN ID of the drive motor
   * @param rotationMotorCanID The CAN ID of the rotation motor
   * @param encoderCanID       The CAN ID of the rotation CANCoder
   * @param canBus             The name of the can bus the devices are connected
   *                           to.
   */
  public Mk4SwerveModuleProSparkFlex(
      ModuleCode moduleCode,
      int driveMotorCanID,
      int rotationMotorCanID,
      int encoderCanID,
      String canBus) {
    super(moduleCode.name() + "SwerveModule");

    this.moduleCode = moduleCode;

    rotationEncoder = new CANcoder(encoderCanID, canBus);
    rotationEncoderConfig = new CANcoderConfiguration();
    //rotationEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; ???
    rotationEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    rotationEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble(getName() + "RotationOffset", 0.0) / 360.0;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    // rotationEncoderSimState = rotationEncoder.getSimState();

    driveMotor = new SparkFlex(driveMotorCanID, SparkLowLevel.MotorType.kBrushless);
    driveMotorConfig.inverted(true);
    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    driveMotorConfig.smartCurrentLimit(100,80);
    ClosedLoopConfig driveMotorPidConfig = driveMotorConfig.closedLoop;
    driveMotorPidConfig.pid(DRIVE_KP, DRIVE_KI, DRIVE_KD);
    driveMotorPidConfig.iZone(DRIVE_I_ZONE);
    driveMotorPidConfig.velocityFF(DRIVE_FEED_FORWARD); // ?
    driveMotorConfig.smartCurrentLimit(100, 80);
    // driveSimState.addSparkMax(driveMotor, 8.0f, 5500.0f);
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    rotationMotor = new SparkFlex(rotationMotorCanID, SparkLowLevel.MotorType.kBrushless);
    rotationMotorConfig.inverted(true);
    rotationMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    ClosedLoopConfig rotationMotorPidConfig = driveMotorConfig.closedLoop;
    ClosedLoopSlot[] rotationSlots = { ClosedLoopSlot.kSlot0, ClosedLoopSlot.kSlot1 };
    for ( ClosedLoopSlot slot : rotationSlots ) {
      rotationMotorPidConfig.pid(ROTATION_KP, ROTATION_KI, ROTATION_KD, slot);
      rotationMotorPidConfig.iZone(ROTATION_I_ZONE, slot);
      rotationMotorPidConfig.velocityFF(ROTATION_FEED_FORWARD, slot);
      rotationMotorPidConfig.maxMotion.maxVelocity(ROTATION_MAX_VELOCITY, slot);
      rotationMotorPidConfig.maxMotion.maxAcceleration(ROTATION_MAX_ACCELERATION, slot);
      rotationMotorPidConfig.maxMotion.allowedClosedLoopError(ROTATION_ERROR, slot);
    }
    rotationMotorConfig.smartCurrentLimit(100, 80);
    // rotationSimState.addSparkMax(rotationMotor, 8.0, 5500.0);
    rotationMotor.configure(rotationMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    rotationAbsoluteSignal = rotationEncoder.getAbsolutePosition();
    rotationAbsoluteVelSignal = rotationEncoder.getVelocity();

    // driveSim = new
    // LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA));
    // rotationSim =
    // new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(ROTATION_KV,
    // ROTATION_KA));

    steerTunable = new TunableSparkPIDController(rotationMotor, ClosedLoopSlot.kSlot1);
    driveTunable = new TunableSparkPIDController(driveMotor, ClosedLoopSlot.kSlot0);

    registerHardware("Drive Motor", driveMotor);
    registerHardware("Rotation Motor", rotationMotor);
    registerHardware("Rotation Encoder", rotationEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + "/DriveTemp", driveMotor.getMotorTemperature());
    SmartDashboard.putNumber(getName() + "/RotationTemp", rotationMotor.getMotorTemperature());
    SmartDashboard.putNumber(getName() + "/DriveOutput", driveMotor.getAppliedOutput());
    SmartDashboard.putNumber(getName() + "/DriveTarget", targetState.speedMetersPerSecond);
    SmartDashboard.putNumber(getName() + "/DriveActual", getDriveVelocityMetersPerSecond());
    SmartDashboard.putNumber(getName() + "/SteerOutput", rotationMotor.getAppliedOutput());
    SmartDashboard.putNumber(getName() + "/SteerTarget", targetState.angle.getDegrees());
    SmartDashboard.putNumber(getName() + "/SteerActual", getAbsoluteRotationDegrees());
    SmartDashboard.putNumber(getName() + "Current", driveMotor.getOutputCurrent());
    // Refresh cached values in background
    StatusSignal.waitForAll(
        0,
        rotationAbsoluteSignal,
        rotationAbsoluteVelSignal);
  }

  public TunablePID getSteerTunablePID() {
    return steerTunable;
  }

  public TunablePID getDriveTunablePID() {
    return driveTunable;
  }

  @Override
  public void simulationPeriodic() {
    // driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // rotationSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // rotationEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // driveSim.setInput(driveSimState.getMotorVoltage());
    // rotationSim.setInput(rotationSimState.getMotorVoltage());

    // driveSim.update(0.02);
    // rotationSim.update(0.02);

    // double driveVel = driveSim.getOutput(0) / DRIVE_METERS_PER_ROTATION;
    // driveSimState.setRotorVelocity(driveVel);
    // driveSimState.addRotorPosition(driveVel * 0.02);

    // double rotationPos = rotationSim.getOutput(0) /
    // ROTATION_DEGREES_PER_ROTATION;
    // double rotationDeltaPos = rotationPos - rotationPositionSignal.getValue();
    // rotationSimState.addRotorPosition(rotationDeltaPos);
    // rotationSimState.setRotorVelocity(rotationDeltaPos / 0.02);
    // rotationEncoderSimState.setRawPosition(rotationSim.getOutput(0) / 360.0);
    // rotationEncoderSimState.setVelocity(getRotationVelocityDegreesPerSecond() /
    // 360.0);
  }

  /**
   * Set the desired state of this module
   *
   * @param desiredState Desired state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    desiredState.optimize(getState().angle);
    this.targetState = desiredState;

    // Don't run the motors if the desired speed is less than 5% of the max
    if (Math.abs(desiredState.speedMetersPerSecond) < DRIVE_MAX_VEL * 0.01) {
      stopMotors();
      return;
    }

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double targetAngle = getRelativeRotationDegrees() + deltaRot;

    REVLibError driveError = driveMotor.getClosedLoopController()
        .setReference(60 * targetState.speedMetersPerSecond / DRIVE_METERS_PER_ROTATION, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    if (driveError != REVLibError.kOk) {
      addFault(
          "[Drive Motor]: Status code: "
              + driveError.name());
    }
    if (Math.abs(deltaRot) > 0.05) {
      REVLibError rotationError = rotationMotor.getClosedLoopController()
          .setReference(targetAngle / ROTATION_DEGREES_PER_ROTATION, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
      if (rotationError != REVLibError.kOk) {
        addFault(
            "[Rotation Motor]: Status code: "
                + rotationError.name());
      }
    } else {
      rotationMotor.stopMotor();
    }
}

  /** Stop all motors */
  public void stopMotors() {
    // NeutralOut request for coast mode
    driveMotor.stopMotor();
    rotationMotor.stopMotor();
  }

  /**
   * Get the position of the drive motor in meters
   *
   * @return How far this module has driven in meters
   */
  public double getDrivePositionMeters() {
    return driveMotor.getEncoder().getPosition() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the relative rotation of this module in degrees.
   *
   * @return Relative rotation
   */
  public double getRelativeRotationDegrees() {
    return rotationMotor.getEncoder().getPosition() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the absolute rotation of this module in degrees.
   *
   * @return Absolute rotation
   */
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValueAsDouble() * 360.0;
  }

  /**
   * Get the velocity of the drive motor in meters/sec
   *
   * @return Drive motor velocity
   */
  public double getDriveVelocityMetersPerSecond() {
    return (driveMotor.getEncoder().getVelocity() * DRIVE_METERS_PER_ROTATION) / 60;
  }

  /**
   * Get the velocity of the rotation motor in deg/sec
   *
   * @return Rotation motor velocity
   */
  public double getRotationVelocityDegreesPerSecond() {
    return (rotationMotor.getEncoder().getVelocity() * ROTATION_DEGREES_PER_ROTATION) / 60;
  }

  /**
   * Get the rotation of this module as a Rotation2d
   *
   * @return Rotation2d for this module
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getAbsoluteRotationDegrees());
  }

  /**
   * Get the current state of this module
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getRotation());
  }

  /**
   * Get the current positions of this module
   *
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPositions() {
    return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
  }

  /**
   * Update the rotation offset for this module. This will assume that the current
   * module position
   * should be the new zero.
   */
  public void updateRotationOffset() {
    double currentOffset = rotationEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue().baseUnitMagnitude()) % 1.0;
    Preferences.setDouble(getName() + "RotationOffset", offset * 360.0);
    rotationEncoderConfig.MagnetSensor.MagnetOffset = offset;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    syncRotationEncoders();
  }

  /**
   * Sync the relative rotation encoder (falcon) to the value of the absolute
   * encoder (CANCoder)
   */
  public void syncRotationEncoders() {
    rotationMotor.getEncoder().setPosition(getAbsoluteRotationDegrees() / ROTATION_DEGREES_PER_ROTATION);
  }

  public void lockModule() {
    double targetAngle = -45;
    if (moduleCode == ModuleCode.FL || moduleCode == ModuleCode.BR) {
      targetAngle = 45;
    }

    targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(targetAngle));
    targetState.optimize(getRotation());

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double angle = getRelativeRotationDegrees() + deltaRot;

    // MAYBE? .setRotorControl
    driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    driveMotor.stopMotor();
    rotationMotor.getClosedLoopController().setReference(angle / ROTATION_DEGREES_PER_ROTATION, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              clearFaults();
              driveMotor.stopMotor();
              rotationMotor.set(0.3);
            },
            this),
        Commands.waitSeconds(0.3),
        Commands.runOnce(
            () -> {
              if (getRotationVelocityDegreesPerSecond() < 20) {
                addFault(
                    "[System Check] Rotation motor encoder velocity measured too slow",
                    false,
                    true);
              }
              if (rotationAbsoluteVelSignal.getValueAsDouble() * 360 < 20) {
                addFault(
                    "[System Check] Absolute encoder velocity measured too slow "
                        + (rotationAbsoluteVelSignal.getValueAsDouble() * 360),
                    false, true);
              }
            },
            this),
        Commands.run(
            () -> {
              double deltaRot = 90 - getAbsoluteRotationDegrees();
              if (deltaRot > 180) {
                deltaRot -= 360;
              } else if (deltaRot < -180) {
                deltaRot += 360;
              }
              double angle = getRelativeRotationDegrees() + deltaRot;
              rotationMotor.getClosedLoopController().setReference(
                  angle / ROTATION_DEGREES_PER_ROTATION, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
            },
            this)
            .withTimeout(1.0),
        Commands.runOnce(
            () -> {
              if (getAbsoluteRotationDegrees() < 70 || getAbsoluteRotationDegrees() > 110) {
                addFault(
                    "[System Check] Rotation Motor did not reach target position", false, true);
              }
            },
            this),
        Commands.runOnce(
            () -> {
              driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
              driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
              driveMotor.set(0.1);
              rotationMotor.stopMotor();
            },
            this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> {
              if (getDriveVelocityMetersPerSecond() < 0.25) {
                addFault("[System Check] Drive motor encoder velocity too slow", false, true);
              }
              driveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
              driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
              driveMotor.stopMotor();
            },
            this),
        Commands.waitSeconds(0.25),
        Commands.run(
            () -> {
              double deltaRot = 0 - getAbsoluteRotationDegrees();
              if (deltaRot > 180) {
                deltaRot -= 360;
              } else if (deltaRot < -180) {
                deltaRot += 360;
              }
              double angle = getRelativeRotationDegrees() + deltaRot;
              rotationMotor.getClosedLoopController().setReference(
                  angle / ROTATION_DEGREES_PER_ROTATION, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
            },
            this)
            .withTimeout(1.0),
        Commands.runOnce(
            () -> {
              if (Math.abs(getAbsoluteRotationDegrees()) > 20) {
                addFault("[System Check] Rotation did not reach target position", false, true);
              }
            },
            this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(this::stopMotors, this));
  }
}
