// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Climber extends AdvancedSubsystem {
  private final SparkFlex climberMotor;
  private final SparkFlexConfig climberMotorConfig = new SparkFlexConfig();
  private final DoubleSolenoid climberPiston;
  private final SparkClosedLoopController climberController;
  private final SingleJointedArmSim physicsSimulation;
  private final SparkFlexSim motorSimulation;
  // Encoder variable
  private final RelativeEncoder climberEncoder;
  private final SparkLimitSwitch climberLimitSwitchLower;
  private final SparkLimitSwitch climberLimitSwitchUpper;

  private Rotation2d climberAbsoluteAngle;

  /** Creates a new Climber. */
  public Climber(final int motor_canid, final int pcmid, final int FORWARDSOLENOID, int REVERSESOLENOID,
      int encoderCanID) {
    climberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, FORWARDSOLENOID, REVERSESOLENOID);
    climberMotor = new SparkFlex(motor_canid, MotorType.kBrushless);
    climberController = climberMotor.getClosedLoopController();

    // Encoder Config
    climberEncoder = climberMotor.getEncoder();

    climberMotorConfig.inverted(false); // just incase :D
    climberMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
    climberMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    climberLimitSwitchLower = climberMotor.getForwardLimitSwitch();
    climberLimitSwitchUpper = climberMotor.getReverseLimitSwitch();
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // climberMotorConfig.smartCurrentLimit(100,80);
    final ClosedLoopConfig climberMotorPidConfig = climberMotorConfig.closedLoop;
    climberMotorPidConfig.pid(Constants.Climber.MOTOR_KP, Constants.Climber.MOTOR_KI, Constants.Climber.MOTOR_KD);
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    physicsSimulation = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.Climber.GEAR_RATIO,
        Constants.Climber.ARM_ANGULAR_MOMENTUM, Constants.Climber.LENGTH_METERS, Constants.Climber.MIN_ANGLE_RADS,
        Constants.Climber.MAX_ANGLE_RADS, false, 0);
    motorSimulation = new SparkFlexSim(climberMotor, DCMotor.getNeoVortex(1));
    physicsSimulation.wouldHitLowerLimit(-3 * Math.PI / 4);
    physicsSimulation.wouldHitUpperLimit(0.0);
  }

  // HEY I ALEADY PUT IN THE STAGE GEAR RATIOS IN THE CONSTANTS!! -- Shirley C. :)
  // -- Tanx

  @Override
  public void periodic() {

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

  /**
   * A method that is used to check that the motors are moving at the right speed.
   */
  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              climberMotor.set(.25);
            }, this),
        Commands.waitSeconds(0.25),
        Commands.runOnce(
            () -> {
              if (((climberEncoder.getVelocity() / 60.0) * Constants.Climber.ARM_ANGULAR_MOMENTUM) < 0.16) {
                addFault("[System Check] Climber Velocity is too slow", false, true);
              }
              climberMotor.stopMotor();
            }, this),
        Commands.waitSeconds(0.25),
        Commands.runOnce(
            () -> {
              climberMotor.set(-0.25);
            }, this),
        Commands.waitSeconds(0.25),
        Commands.runOnce(
            () -> {
              if (((climberEncoder.getVelocity() / 60.0) * Constants.Climber.ARM_ANGULAR_MOMENTUM) > -0.16) {
                addFault("[System Check] Climber Velocity is too slow", false, true);
              }
              climberMotor.stopMotor();
            }, this));
  }

  /**
   * This method will move the climber to the required angle
   * 
   * @returns void
   * @param angle
   */
  public void setClimberAngle(Rotation2d angle) {
    climberAbsoluteAngle = angle;
    double armRotation = (angle.getRotations());
    double motorRotation = armRotation / Constants.Climber.GEAR_RATIO;
    climberController.setReference(motorRotation, ControlType.kPosition);
  }

  /**
   * This method is intended to give the user the current target loctation of the
   * climber
   * 
   * @returns target angle for the climber
   */
  public Rotation2d getCurrentTarget() {
    return climberAbsoluteAngle;
  }

  /**
   * This method will get the current angle of the Climbing mechanism
   * 
   * @returns the current angle of the climber
   * 
   */
  public void runClimberMotor() {
    climberController.setReference(-500.0, ControlType.kVelocity);
  }

  public void reverseClimberMotor() {
    climberController.setReference(500.0, ControlType.kVelocity);
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
   * This method will open the claw
   */
  public void toggleClaw(){
    climberPiston.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * This method will close the claw
   */
  public void detoggleClaw(){
    climberPiston.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * This method will set the Climber back to the default position
   */
  // TODO not working, need to confirm where the default position is with climbing
  // team
  public void setDefaultPosition() {
    setClimberAngle(Rotation2d.fromRadians(0));
  }

  public Command getCalibrateCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          toggleClaw();
          climberMotor.set(-1.0);
        }, this),
        Commands.waitUntil(() -> hitUpperLimit()),
        Commands.runOnce(() -> {
          climberMotor.stopMotor();
          climberMotor.getEncoder()
              .setPosition(Rotation2d.fromRadians(3 * Math.PI / 4).getRotations() / Constants.Climber.GEAR_RATIO);
        }, this));
  }

  // Prepare the jaw Commands
  public Command getOpenCommand() {
    return Commands.runOnce(() -> {
      detoggleClaw();
    }, this);
  }

  public Command getPrepareCommand() {
    return Commands.sequence(Commands.runOnce(() -> {
      toggleClaw();
      setClimberAngle(Rotation2d.fromDegrees(0));
    },
        this),
        Commands.waitUntil(() -> {
          return ((getCurrentAngle().getDegrees() + 90) > 0);
        }),
        Commands.runOnce(() -> {
          detoggleClaw();
        }));
  }

  public Command getRotateCommand(Rotation2d desiredAngle) {
    return Commands.runOnce(() -> {
      setClimberAngle(desiredAngle);
    }, this);
  }

  // Clamp jaw
  public Command getCloseCommand() {
    return Commands.runOnce(() -> {
      toggleClaw();
    }, this);
  }

  // Rotate thy clamped jaw
  public Command getClimbCommand() {
    return Commands.runOnce(() -> {
      setClimberAngle(Rotation2d.fromDegrees(130));
    }, this);
  }

  public Command toggleClimberCommand() {
    return Commands.runOnce(() -> {
      toggleClaw();
    }, this);
  }

  // stow climber
  public Command getStowCommand() {
    return Commands.runOnce(() -> {
      setClimberAngle(Rotation2d.fromDegrees(-135));
      toggleClaw();
    },

        this);
  }

  public Command getPrepareCommandS() {
    return Commands.runOnce(() -> {
      toggleClaw();
      setClimberAngle(Rotation2d.fromDegrees(0));
    },
        this);
  }

  public Command getRotateCommandS(Rotation2d desiredAngle) {
    return Commands.runOnce(() -> {
      setClimberAngle(desiredAngle);
    }, this);
  }

  public Command raiseClawCommand() {
    return Commands.runOnce(() -> {
      if (!hitUpperLimit())
        runClimberMotor();
      else
        stopClimberMotor();
    }, this);
  }
  
  public Command runClawMotorUpCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runClimberMotor();
            }, this),
        Commands.waitSeconds(5),

        Commands.runOnce(
            () -> {
              stopClimberMotor();
            }, this));
  }

  public Command runClawMotorOneWayThenOther() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runClimberMotor();
            }, this),
        Commands.waitSeconds(5),

        Commands.runOnce(
            () -> {
              stopClimberMotor();
            }, this),
        Commands.runOnce(
            () -> {
              reverseClimberMotor();
            }, this),
        Commands.waitSeconds(5),

        Commands.runOnce(
            () -> {
              stopClimberMotor();
            }, this));
          }

}

// notes or todo, configure 2 limit switches, double solenoid