// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public final class Climber extends AdvancedSubsystem {
  private final SparkFlex climberMotor;
  private final SparkFlexConfig climberMotorConfig = new SparkFlexConfig();
  private final DoubleSolenoid climberPiston;
  private final CANcoder climberAbsoluteEncoder;
  private final CANcoderConfiguration climberEncoderConfig;
  private final SparkClosedLoopController climberController;
  private final SingleJointedArmSim physicsSimulation;
  private final SparkFlexSim motorSimulation;
  // Encoder variable
  private final RelativeEncoder climberEncoder;
  private final SparkLimitSwitch climberLimitSwitchLower;
  private final SparkLimitSwitch climberLimitSwitchUpper;
  private final StatusSignal<Angle> absoluteEncoderAngle;
  private final StatusSignal<AngularVelocity> absoluteEncoderVel;

  private Rotation2d climberTargetAngle = new Rotation2d();

  /** Creates a new Climber. */
  public Climber(final int MOTOR_CANID, final int pcmid, final int FORWARDSOLENOID, int REVERSESOLENOID,
      int ENCODER_CANID) {
    climberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARDSOLENOID, REVERSESOLENOID);
    climberMotor = new SparkFlex(MOTOR_CANID, MotorType.kBrushless);
    climberAbsoluteEncoder = new CANcoder(ENCODER_CANID);
    climberController = climberMotor.getClosedLoopController();

    // Encoder Config
    climberEncoder = climberMotor.getEncoder();
    climberEncoderConfig = new CANcoderConfiguration();
    climberEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO
    climberEncoderConfig.MagnetSensor.MagnetOffset = Preferences.getDouble("ClimberRotationalOffset", 0) / 360.0;
    climberEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO
    climberAbsoluteEncoder.getConfigurator().apply(climberEncoderConfig);
    climberAbsoluteEncoder.getAbsolutePosition().refresh();
    absoluteEncoderAngle = climberAbsoluteEncoder.getAbsolutePosition();
    absoluteEncoderVel = climberAbsoluteEncoder.getVelocity();

    syncEncoders();

    climberMotorConfig.inverted(false); // just incase :D
    climberMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
    climberMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    climberLimitSwitchLower = climberMotor.getReverseLimitSwitch();
    climberLimitSwitchUpper = climberMotor.getForwardLimitSwitch();
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // climberMotorConfig.smartCurrentLimit(100,80);
    final ClosedLoopConfig climberMotorPidConfig = climberMotorConfig.closedLoop;
    climberMotorPidConfig.pidf(Constants.Climber.MOTOR_KP, Constants.Climber.MOTOR_KI, Constants.Climber.MOTOR_KD,
        Constants.Climber.MOTOR_FF, ClosedLoopSlot.kSlot0); //position
        climberMotorPidConfig.pidf(Constants.Climber.MOTOR_MAX_KP, Constants.Climber.MOTOR_MAX_KI, Constants.Climber.MOTOR_MAX_KD,
        Constants.Climber.MOTOR_MAX_FF, ClosedLoopSlot.kSlot1); //maxmotion
    climberMotorPidConfig.maxMotion.maxVelocity(Constants.Climber.MOTOR_MAX_VELOCITY);
    climberMotorPidConfig.maxMotion.maxAcceleration(Constants.Climber.MOTOR_MAX_ACCEL);
    climberMotorPidConfig.maxMotion.allowedClosedLoopError(2.0); //TODO
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    physicsSimulation = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.Climber.GEAR_RATIO,
        Constants.Climber.ARM_ANGULAR_MOMENTUM, Constants.Climber.LENGTH_METERS,
        Constants.Climber.MIN_ANGLE.getRadians(),
        Constants.Climber.MAX_ANGLE.getRadians(), false, 0);
    motorSimulation = new SparkFlexSim(climberMotor, DCMotor.getNeoVortex(1));
    physicsSimulation.wouldHitLowerLimit(-3 * Math.PI / 4);
    physicsSimulation.wouldHitUpperLimit(0.0);
    SmartDashboard.putData("Open Claw", getOpenClawCommand());
    SmartDashboard.putData("Close Claw", getCloseClawCommand());
    SmartDashboard.putData("Set Climber to -90", setClimberNeg90());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/AbsoluteAngle", absoluteEncoderAngle.getValueAsDouble() * 360);
    SmartDashboard.putNumber("Climber/RelativeAngle", getCurrentAngle().getDegrees());
    SmartDashboard.putNumber("Climber/TargetAngle", getCurrentTarget().getDegrees());

    StatusSignal.waitForAll(
        0,
        absoluteEncoderAngle,
        absoluteEncoderVel);
  }

  @Override
  public void simulationPeriodic() {
    // Simulates gravity for the elevator
    physicsSimulation.setInputVoltage(climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    physicsSimulation.update(0.02);
    // Sets a variable for motor speed and sets the Simulation Motor's Velocity to
    // it.
    motorSimulation.getForwardLimitSwitchSim().setPressed(physicsSimulation.hasHitUpperLimit());
    motorSimulation.getReverseLimitSwitchSim().setPressed(physicsSimulation.hasHitLowerLimit());

    final double motorSpeed = ((physicsSimulation.getVelocityRadPerSec() / Constants.Climber.GEAR_RATIO) * 60)
        / (2 * Math.PI);
    motorSimulation.iterate(motorSpeed, RobotController.getBatteryVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(motorSimulation.getMotorCurrent()));
  }

  public void updateEncoders() {
    double currentOffset = climberEncoderConfig.MagnetSensor.MagnetOffset;
    absoluteEncoderAngle.refresh();
    double offset = (currentOffset - absoluteEncoderAngle.getValue().magnitude()) % 1.0;
    Preferences.setDouble("ClimberRotationalOffset", offset * 360.0);
    climberEncoderConfig.MagnetSensor.MagnetOffset = offset;
    climberAbsoluteEncoder.getConfigurator().apply(climberEncoderConfig);
    syncEncoders();
  }

  /**
   * This method will move the climber to the required angle
   * 
   * @returns void
   * @param angle
   */
  public void setClimberAngle(Rotation2d angle) {
    climberTargetAngle = angle;
    Rotation2d currentAngle = Rotation2d.fromRotations(absoluteEncoderAngle.getValueAsDouble());
    double armRotation = (angle.getRotations());
    double difference = armRotation - currentAngle.getRotations();
    double motorRotation = difference / Constants.Climber.GEAR_RATIO;
    double actualTarget = motorRotation + climberEncoder.getPosition();
    
    if (Math.abs(actualTarget - getCurrentAngle().getDegrees()) > 2) {
    climberController.setReference(actualTarget, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    }
    else {
      climberController.setReference(actualTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  }

  /**
   * This method is intended to give the user the current target location of the
   * climber
   * 
   * @returns target angle for the climber
   */
  public Rotation2d getCurrentTarget() {
    return climberTargetAngle;
  }

  /**
   * This method will get the current angle of the Climbing mechanism
   * 
   * @returns the current angle of the climber
   * 
   */
  public void runClimberMotor() {
    climberMotor.set(.2);
    // climberController.setReference(-500.0, ControlType.kVelocity);
  }

  public void reverseClimberMotor() {
    climberMotor.set(-.2);
    // climberController.setReference(500.0, ControlType.kVelocity);
  }

  public boolean hitLowerLimit() {
    // Will be true when algae handler has algae
    return climberLimitSwitchLower.isPressed();
  }

  public boolean hitUpperLimit() {
    return climberLimitSwitchUpper.isPressed();
  }

  public void stopClimberMotor() {
    climberMotor.stopMotor();
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(climberEncoder.getPosition() * Constants.Climber.GEAR_RATIO);
  }

  // methods to close and open claw, and stop
  public boolean isClawOpen() {
    return climberPiston.get() == DoubleSolenoid.Value.kReverse;
  }

  /**
   * This method will close the claw
   */
  public void closeClaw() {
    climberPiston.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * This method will open the claw was detoggle
   */
  public void openClaw() {
    if (getCurrentAngle().getDegrees() > -80.0) {
      climberPiston.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void syncEncoders() {
    climberMotor.getEncoder()
        .setPosition(absoluteEncoderAngle.getValueAsDouble() / Constants.Climber.GEAR_RATIO);
  }

  /**
   * This method will set the Climber back to the default position
   */
  // TODO not working, need to confirm where the default position is with climbing
  // team
  public void setDefaultPosition() {
    setClimberAngle(Rotation2d.fromRadians(0));
  }

  public Command getCalibrateCommand(boolean isReverse) {
    return Commands.sequence(
        Commands.runOnce(() -> {
          double speed = 0.2;
          if (isReverse)
            speed *= -1.0;
          climberMotor.set(speed);
        }, this),
        Commands.waitUntil(() -> {
          if (isReverse)
            return hitLowerLimit();
          else
            return hitUpperLimit();
        }),
        Commands.runOnce(() -> {
          climberMotor.stopMotor();
          if (isReverse) {
            climberMotor.getEncoder()
                .setPosition(Constants.Climber.MIN_ANGLE.getRotations() / Constants.Climber.GEAR_RATIO);
            climberAbsoluteEncoder.setPosition(Constants.Climber.MIN_ANGLE.getRotations()); // TODO Sync Motors instead?
          } else {
            updateEncoders(); // TODO Sync Motors instead?
          }
        }, this));
  }

  // Rotation2d.fromRadians(3 * Math.PI / 4).getRotations() /
  // Constants.Climber.GEAR_RATIO

  // Prepare the jaw Commands
  public Command getOpenClawCommand() {
    return Commands.runOnce(() -> {
      if (getCurrentAngle().getDegrees() < -80)
        openClaw();
    }, this);
  }

  public Command getClimberAngle(Rotation2d angle) {
    return Commands.runOnce(() -> {
      getCurrentTarget();
    }, this);
  }

  public Command setClimberNeg90() {
    return Commands.runOnce(() -> {
      setClimberAngle(Rotation2d.fromDegrees(-90));
    }, this);
  }

  public Command getPrepareCommand() {
    return Commands.sequence(Commands.runOnce(() -> {
      closeClaw();
    },
        this),
        Commands.parallel(
            Commands.sequence(
                Commands.waitUntil(() -> {
                  return ((getCurrentAngle().getDegrees() + 70) > 0);
                }),
                Commands.runOnce(() -> {
                  openClaw();
                })),
            Commands.run(
                () -> {
                  setClimberAngle(Rotation2d.fromDegrees(0));
                })));
  }

  // Clamp jaw
  public Command getCloseClawCommand() {
    return Commands.runOnce(() -> {
      closeClaw();
    }, this);
  }

  // Rotate the clamped jaw
  public Command runClimberMotorCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          runClimberMotor();
        }, this),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> {
          stopClimberMotor();
        }, this));

  }

  public Command reverseClimbMotorCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          reverseClimberMotor();
        }, this),
        Commands.waitSeconds(.5),
        Commands.runOnce(() -> {
          stopClimberMotor();
        }, this));
  }

  public Command getPrepareCommandS() {
    return Commands.runOnce(() -> {
      closeClaw();
      setClimberAngle(Rotation2d.fromDegrees(0));
    }, this);
  }

  public Command getRotateCommandS(Rotation2d desiredAngle) {
    return Commands.runOnce(() -> {
      setClimberAngle(desiredAngle);
    }, this);
  }

  public Command climbCommand(Rotation2d angle) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              closeClaw();
            }, this),
        Commands.run(
            () -> {
              setClimberAngle(angle);
            }, this));
  }

  /**
   * A method that is used to check that the motors are moving at the right speed.
   */
  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              climberMotor.set(.2);
            }, this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> {
              if (((climberEncoder.getVelocity() / 60.0) * Constants.Climber.ARM_ANGULAR_MOMENTUM) < 0.16) {
                addFault("[System Check] Climber Velocity is too slow", false, true);
              }
              climberMotor.stopMotor();
            }, this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> {
              climberMotor.set(-0.2);
            }, this),
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> {
              if (((climberEncoder.getVelocity() / 60.0) * Constants.Climber.ARM_ANGULAR_MOMENTUM) > -0.16) {
                addFault("[System Check] Climber Velocity is too slow", false, true);
              }
              climberMotor.stopMotor();
            }, this));
  }
}