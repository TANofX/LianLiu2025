// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Climber extends AdvancedSubsystem {
  private final SparkFlex climberMotor;
  private final SparkFlexConfig climberMotorConfig = new SparkFlexConfig();
  private final DoubleSolenoid climberPiston;
  private final SparkClosedLoopController climbercontroller;
  private final SingleJointedArmSim physicsSimulation;
  private final SparkFlexSim motorSimulation;
  // Encoder variable
  private final RelativeEncoder climberEncoder;
  
  // Absolute Encoder variables
  private final CANcoder climberEncoderAbsolute;
  private final CANcoderConfiguration climberEncoderConfig;
  private final CANcoderSimState climberEncoderSimState;
  private final StatusSignal<Angle> climberEncoderSignalA; // TODO
  private final StatusSignal<AngularVelocity> climberEncoderSignalB; //TODO
  private Rotation2d climberAbsoluteAngle;

  /** Creates a new Climber. */
  public Climber(final int motor_canid, final int pcmid, final int solonoidid, int encoderCanID) {
    climberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1,2);
    climberMotor = new SparkFlex(motor_canid, MotorType.kBrushless);
    climbercontroller = climberMotor.getClosedLoopController();

    // Encoder Config
    climberEncoder = climberMotor.getEncoder();

    // Absolute Encoder Config
    climberEncoderAbsolute = new CANcoder(encoderCanID);
    climberEncoderConfig = new CANcoderConfiguration();
    climberEncoderSignalA = climberEncoderAbsolute.getAbsolutePosition();
    climberEncoderSignalB = climberEncoderAbsolute.getVelocity();
    climberEncoderSimState = climberEncoderAbsolute.getSimState();
    
    // Holds the angle that the climber starts at in comparison to position zero
    climberAbsoluteAngle = Rotation2d.fromDegrees(climberEncoderSignalA.getValueAsDouble() * 180);

    climberMotorConfig.inverted(false); // just incase :D
    climberMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
    climberMotorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
    climberMotorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //climberMotorConfig.smartCurrentLimit(100,80);
    final ClosedLoopConfig climberMotorPidConfig = climberMotorConfig.closedLoop;
    climberMotorPidConfig.pid(Constants.Climber.MOTOR_KP, Constants.Climber.MOTOR_KI, Constants.Climber.MOTOR_KD);
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    physicsSimulation = new SingleJointedArmSim(DCMotor.getNeoVortex(1), Constants.Climber.GEAR_RATIO, Constants.Climber.ARM_ANGULAR_MOMENTUM, Constants.Climber.LENGTH_METERS, Constants.Climber.MIN_ANGLE_RADS, Constants.Climber.MAX_ANGLE_RADS, false, 0);
    motorSimulation = new SparkFlexSim(climberMotor, DCMotor.getNeoVortex(1));
  }

  // HEY I ALEADY PUT IN THE STAGE GEAR RATIOS IN THE CONSTANTS!! -- Shirley C. :) -- Tanx


  @Override
  public void periodic() {
    // TODO This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Simulates gravity for the elevator
    physicsSimulation.setInputVoltage(climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    physicsSimulation.update(0.02);
    // Sets a variable for motor speed and sets the Simulation Motor's Velocity to it.
    final double motorSpeed = ((physicsSimulation.getVelocityRadPerSec() / Constants.Climber.GEAR_RATIO) * 60) / (2 * Math.PI);
    motorSimulation.iterate(motorSpeed, RobotController.getBatteryVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(motorSimulation.getMotorCurrent()));
  }
  /** A method that is used to check that the motors are moving at the right speed.*/
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
          }, this)
    );
  }
  /**
   * This method will move the climber to the required angle
   * @returns void
   * @param angle
   */
  public void setClimberAngle(Rotation2d angle) {
    climberAbsoluteAngle = angle;
    double armRotation = (angle.getRotations()); 
    double motorRotation = armRotation / Constants.Climber.GEAR_RATIO;
    climbercontroller.setReference(motorRotation, ControlType.kPosition);
  }
  /**
   * This method is intended to give the user the current target loctation of the climber
   * @returns target angle for the climber
  */
  public Rotation2d getCurrentTarget() {
    return climberAbsoluteAngle;
  }
  /**
   * This method will get the current angle of the Climbing mechanism
   * @returns the current angle of the climber
   */
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(climberEncoderSignalA.getValueAsDouble()/2.0);
  }   

  //methods to close and open claw, and stop
  public boolean isClawOpen(){
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
  // TODO not working, need to confirm where the default position is with climbing team
  public void setDefaultPosition() {
    setClimberAngle(Rotation2d.fromRadians(0)); 
  }

  //Prepare the jaw Commands 
  public Command getOpenCommand(){
    return Commands.runOnce(()->{detoggleClaw();},this);
  }
  public Command getPrepareCommand(){
    return Commands.runOnce(()->{
      toggleClaw();
      setClimberAngle(Rotation2d.fromDegrees(0));},
      this);
  }
  public Command getRotateCommand(Rotation2d desiredAngle){
    return Commands.runOnce(()->{setClimberAngle(desiredAngle);},this);
  }

  /**
   * A command to clamp the jaw
   * @return
   */ 
  public Command getCloseCommand(){
    return Commands.runOnce(()->{toggleClaw();},this);
  }

  /**
   * A command to rotate thy clamped jaw 
   * @return
   */ 
  public Command getClimbCommand(){
    return Commands.runOnce(()->{setClimberAngle(Rotation2d.fromDegrees(130));},this);
  }

  /**
   * A Command to stow climber
   * @return
   */
  public Command getStowCommand(){
    return Commands.runOnce(()->{toggleClaw();},this);
  }
  /**
   * This command is unclear sowwy - Jon
   * @return
   */
  public Command getPrepareCommandS(){
    return Commands.runOnce(()->{
      toggleClaw();
      setClimberAngle(Rotation2d.fromDegrees(130));},
      this);
  }
  /**
   * A command to set the angle to the desired angle
   * @param desiredAngle
   * @return
   */
  public Command getRotateCommandS(Rotation2d desiredAngle){
    return Commands.runOnce(()->{setClimberAngle(desiredAngle);},this);
  }


}

//notes or todo, configure 2 limit switches, double solenoid