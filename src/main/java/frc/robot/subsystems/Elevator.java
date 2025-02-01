// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Elevator extends AdvancedSubsystem {
  private SparkFlex elevatorMotor;
  private SparkFlexSim elevatorMotorSim;
  private SparkClosedLoopController elevatorController;
  private RelativeEncoder elevatorEncoder;
  private boolean calibrated = false;

  // This configures a simulated elevator system including gravity in the calculations.
  // We use this simulation in the simulatePeriodic() method to update the simulated state.
  private final ElevatorSim elevatorPhysicsSim = 
    new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getNeoVortex(1),
                                                        Constants.Elevator.ELEVATOR_MASS,
                                                        Constants.Elevator.METERS_PER_MOTOR_REVOLUTION / (2 * Math.PI),
                                                        Constants.Elevator.GEAR_RATIO),
                    DCMotor.getNeoVortex(1),
                    Constants.Elevator.MIN_HEIGHT_METERS,
                    Constants.Elevator.MAX_HEIGHT_METERS,
                    true,
                    Constants.Elevator.STARTING_HEIGHT_METERS);

  //Constructor for the Motor, puts motor into brake mode
  public Elevator(int elevatorCanID) {
    super("Elevator");
    elevatorMotor = new SparkFlex(elevatorCanID, MotorType.kBrushless);
    SparkFlexConfig newConfig = new SparkFlexConfig();
    newConfig.idleMode(IdleMode.kBrake);

    // Configure the closed loop controller. This includes the  PIDF gains and the max motion settings.
    newConfig.closedLoop.pidf(Constants.Elevator.P, Constants.Elevator.I, Constants.Elevator.D, Constants.Elevator.FF, ClosedLoopSlot.kSlot0);
    newConfig.closedLoop.maxMotion.allowedClosedLoopError(0.01, ClosedLoopSlot.kSlot0);
    newConfig.closedLoop.maxMotion.maxAcceleration(Constants.Elevator.MAX_ACCELERATION, ClosedLoopSlot.kSlot0);
    newConfig.closedLoop.maxMotion.maxVelocity(Constants.Elevator.MAX_VELOCITY, ClosedLoopSlot.kSlot0);
    newConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

    elevatorMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorController = elevatorMotor.getClosedLoopController();

    // Configure the motor simulation using the REV Robotics Spark Flex motor model.
    elevatorMotorSim = new SparkFlexSim(elevatorMotor, DCMotor.getNeoVortex(1));
    elevatorMotorSim.setPosition(Constants.Elevator.STARTING_HEIGHT_METERS / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION);
    
    registerHardware("Elevator/Motor", elevatorMotor);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Motor Velocity", elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Position", getElevation());
    
    
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Calculate the input voltage for the elevator simulation. The appliedVoltage is calculated by the SparkFlex simulation.
    // The RobotController battery voltage is simulated in the RoboRioSim class.
    double inputVoltage = elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("Elevator Simulation/Simulated Voltage", inputVoltage);
    SmartDashboard.putNumber("Elevator Simulation/Motor Position", elevatorMotorSim.getPosition());
    SmartDashboard.putNumber("Elevator Simulation/Height", this.getElevation());
    SmartDashboard.putNumber("Elevator Simulation/Simulated Elevator Velocity", elevatorPhysicsSim.getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Elevator Simulation/Simulated Elevator Height", elevatorPhysicsSim.getPositionMeters());

    // Update the input voltage for the elevator simulation and update the simulation.
    elevatorPhysicsSim.setInput(inputVoltage);
    elevatorPhysicsSim.update(0.020);

    // Compute the motor velocity from the elevator simulation state. This will be used to update the SparkFlex motor simulation.
    double motorVelocity = (elevatorPhysicsSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION) * 60.0;
    SmartDashboard.putNumber("Elevator Simulation/Simulated Motor Velocity", motorVelocity);

    // Simulate limit switch behavior
    elevatorMotorSim.getReverseLimitSwitchSim().setPressed(elevatorPhysicsSim.hasHitLowerLimit());
    elevatorMotorSim.getForwardLimitSwitchSim().setPressed(elevatorPhysicsSim.hasHitUpperLimit());

    // Iterate the motor simulation
    elevatorMotorSim.iterate(motorVelocity, RobotController.getBatteryVoltage(), 0.020);

    // Update the RoboRioSim voltage to the default battery voltage based on the elevator motor current.
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorMotorSim.getMotorCurrent()));
  }

  // Check to make sure that the elevator moves at the anticipated speeds up and down
  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
      Commands.runOnce(
        () -> {
          elevatorMotor.set(0.25);
        }, this),
      Commands.waitSeconds(5),
      Commands.runOnce(
        () -> {
          if (((elevatorEncoder.getVelocity() / 60.0) * Constants.Elevator.METERS_PER_MOTOR_REVOLUTION) < 0.16) {
            addFault("[System Check] Elevator velocity too slow", false, true);
          }
          elevatorMotor.stopMotor();
        }, this),
      Commands.waitSeconds(0.25),
      Commands.runOnce(
        () -> {
          elevatorMotor.set(-0.25);
        }, this),
      Commands.waitSeconds(0.25),
      Commands.runOnce(
        () -> {
          if (((elevatorEncoder.getVelocity() / 60.0) * Constants.Elevator.METERS_PER_MOTOR_REVOLUTION) > -0.16) {
            addFault("[System Check] Elevator velocity too slow", false, true);
          }
          elevatorMotor.stopMotor();
        }, this)
    );
  }

  //A method to change the Height of the elevator to the required height
  public void toHeightMeters(double amount) {
    double rotations = amount / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION;

    elevatorController.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  //A method to check the Elevators current height
  public double getElevation() {
    return elevatorEncoder.getPosition() * Constants.Elevator.METERS_PER_MOTOR_REVOLUTION;
  }

  public void stop() {
      elevatorMotor.stopMotor();
  }

  // Calibration Command to reset the encoder to a known position
  public Command getCalibrationCommand() {
    return Commands.sequence(
      Commands.runOnce(
        () -> {
          if (!calibrated) elevatorMotor.set(-0.25);
        }, this),
      Commands.waitUntil(
        () -> {
          return (calibrated || elevatorMotor.getReverseLimitSwitch().isPressed());
        }
      ),
      Commands.runOnce(
        () -> {
          if (!calibrated && elevatorMotor.getReverseLimitSwitch().isPressed()) {
            elevatorEncoder.setPosition(Constants.Elevator.MIN_HEIGHT_METERS / Constants.Elevator.METERS_PER_MOTOR_REVOLUTION);
            calibrated = true;
          }
        }, this)
    );
  }

  // Get a command instance that will move the elevator to a target height
  public Command getElevatorHeightCommand(double targetHeightMeters) {
    if (targetHeightMeters == Constants.Elevator.MIN_HEIGHT_METERS) {
      return Commands.sequence(
        Commands.runOnce(
          () -> {
            toHeightMeters(targetHeightMeters);
          }, this),
        Commands.waitUntil(() -> {
          return Math.abs(targetHeightMeters - getElevation()) < 0.0025;
        }),
        Commands.runOnce(
          () -> {
            stop();
          }, this)
      );
    }
    else {
      return Commands.sequence(
        Commands.runOnce(
        () -> {
          toHeightMeters(targetHeightMeters);
        }, this),
        Commands.waitUntil(() -> {
          return Math.abs(targetHeightMeters - getElevation()) < 0.0025;
        })
      );
    }
  }
}
