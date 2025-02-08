// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

/** Creates a new CoralHandler. */
public class CoralHandler extends AdvancedSubsystem {
  private final SparkFlex outtakeMotor;
  private final SparkLimitSwitch coralLimitSwitch;
  private final RelativeEncoder outtakeEncoder;
  private final SparkFlexSim coralHandlerOuttakeSim;

  private final CoralHandlerWrist horizontalWrist;
  private final CoralHandlerWrist verticalWrist;

  // Creation of Flywheel Simulation for the simulation of the outtakeMotor
  private final FlywheelSim coralHandlerOuttakePhysicsSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 
                                          Constants.CoralHandler.outtakeJKgMetersSquared, 
                                          Constants.CoralHandler.outtakeMotorGearing),
      DCMotor.getNeoVortex(1), 
      Constants.CoralHandler.outtakeMotorGearing);

  public CoralHandler(int outtakeMotorID, int horizontalMotorID, int verticalMotorID, int horizontalAbsoluteEncoderID,
      int verticalAbsoluteEncoderID) {
    super("CoralHandler");
    // Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
    outtakeMotor = new SparkFlex(outtakeMotorID, MotorType.kBrushless);

    horizontalWrist = new CoralHandlerWrist(
            "Horizontal",
            horizontalMotorID,
            horizontalAbsoluteEncoderID,
            Constants.CoralHandler.horizontalGearRatio,
            Constants.CoralHandler.horizontalMotorP,
            Constants.CoralHandler.horizontalMotorI,
            Constants.CoralHandler.horizontalMotorD,
            Constants.CoralHandler.horizontalMotorFeedForward,
            Constants.CoralHandler.horizontalMotorIZone,
            Constants.CoralHandler.horizontalMotorMinVelocity,
            Constants.CoralHandler.horizontalMotorMaxVelocity,
            Constants.CoralHandler.horizontalMotorMaxAccleration,
            Constants.CoralHandler.horizontalMotorClosedLoopError,
            Type.kNormallyOpen,
            Constants.CoralHandler.horizontalMinAngle,
            Constants.CoralHandler.horizontalMaxAngle,
            Constants.CoralHandler.horizontalJKgMetersSquared,
            Constants.CoralHandler.coralEndEffectorLength,
            Constants.CoralHandler.horizontalStartingAngleInRadians
            );
    horizontalWrist.registerSystemCheckWithSmartDashboard();
    verticalWrist = new CoralHandlerWrist(
            "Vertical",
            verticalMotorID,
            verticalAbsoluteEncoderID,
            Constants.CoralHandler.verticalGearRatio,
            Constants.CoralHandler.verticalMotorP,
            Constants.CoralHandler.verticalMotorI,
            Constants.CoralHandler.verticalMotorD,
            Constants.CoralHandler.verticalMotorFeedForward,
            Constants.CoralHandler.verticalMotorIZone,
            Constants.CoralHandler.verticalMotorMinVelocity,
            Constants.CoralHandler.verticalMotorMaxVelocity,
            Constants.CoralHandler.verticalMotorMaxAccleration,
            Constants.CoralHandler.verticalMotorClosedLoopError,
            Type.kNormallyOpen,
            Constants.CoralHandler.verticalMinAngle,
            Constants.CoralHandler.verticalMaxAngle,
            Constants.CoralHandler.verticalJKgMetersSquared,
            Constants.CoralHandler.coralEndEffectorLength,
            Constants.CoralHandler.verticalStartingAngleInRadians
    );


    verticalWrist.registerSystemCheckWithSmartDashboard();

    outtakeEncoder = outtakeMotor.getEncoder();
    coralLimitSwitch = outtakeMotor.getForwardLimitSwitch(); // TODO forward or reverse limit switch?

    // Using SparkFlexConfig to create needed parameters for the outtakeMotor
    SparkFlexConfig outtakeConfig = new SparkFlexConfig();
    outtakeConfig.inverted(false);
    outtakeConfig.idleMode(IdleMode.kBrake);
    outtakeMotor.configure(outtakeConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Creation of CoralHandler motor simulation
    coralHandlerOuttakeSim = new SparkFlexSim(outtakeMotor, DCMotor.getNeoVortex(1));

    // Register Hardware
    registerHardware("Coral Intake/Outtake Motor", outtakeMotor);
  }

  @Override
  public void simulationPeriodic() {
    // Simulates the input voltage of the motors (battery)
    double outtakeInputVoltage = coralHandlerOuttakeSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    
    // Simulation limit switch is set to false
    coralHandlerOuttakeSim.getForwardLimitSwitchSim().setPressed(false);

    // Sets the simulation input velocities based on the voltages above
    coralHandlerOuttakePhysicsSim.setInput(outtakeInputVoltage);

    // Simulates time by updating the time
    coralHandlerOuttakePhysicsSim.update(0.02);

    // Calculating the simulation velocity based on known values
    double outtakeMotorVelocity = coralHandlerOuttakePhysicsSim.getAngularVelocityRPM()
        / Constants.CoralHandler.outtakeMotorGearing;
    

    // Creation of the motor simulations
    coralHandlerOuttakeSim.iterate(outtakeMotorVelocity, RobotController.getBatteryVoltage(), 0.02);
    
    // Creation of the absolute encoder simulations
    
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(coralHandlerOuttakeSim.getMotorCurrent(),
        horizontalWrist.sim.getMotorCurrent(), verticalWrist.sim.getMotorCurrent()));
  }

  /**
   * Stops motor for the coral end effector intake/outtake motor. Sets motor speed
   * to zero.
   */
  public void stopOuttakeMotor() {
    outtakeMotor.stopMotor();
  }

  /**
   * Stops motor for the coral end effector horizontal motor. Sets motor speed to
   * zero.
   */
  public void stopHorizontalMotor() {
    horizontalWrist.stopMotor();
  }

  /**
   * Stops motor for the coral end effector vertical motor. Sets motor speed to
   * zero.
   */
  public void stopVerticalMotor() {
    verticalWrist.stopMotor();
  }

  /**
   * Stops all motors for the coral end effector. Sets all motor speeds to zero.
   */
  public void stopMotors() {
    outtakeMotor.stopMotor();
    stopHorizontalMotor();
    stopVerticalMotor();
  }

  /**
   * A boolean that will return true or false based on if coral end effector's
   * limit switch is hit.
   * 
   * @return true or false.
   */
  public boolean hasCoral() {
    return coralLimitSwitch.isPressed();
  }

  /**
   * Sets coral intake/outtake motor to a specified speed to shoot the coral out.
   * 
   * @param outtakeMotorSpeed Speed to set to be able outtake the coral using
   *                          coral end effector intake/outtake motor.
   */
  public void runOuttakeMotor(double outtakeMotorSpeed) {
    outtakeMotor.set(outtakeMotorSpeed);
  }

  /**
   * Sets the horizontal positioning of the coral end effector to a specified
   * angle.
   * 
   * @param targetAngle The desired horizontal angle (degrees) for the coral end
   *                    effector.
   */
  public void setHorizontalAngle(Rotation2d targetAngle) {
    horizontalWrist.setAngle(targetAngle);
  }

  /**
   * Sets the verical positioning of the coral end effector to a specified angle.
   * 
   * @param targetAngle The desired vertical angle (degrees) for the coral end
   *                    effector.
   */
  public void setVerticalAngle(Rotation2d targetAngle) {
    verticalWrist.setAngle(targetAngle);
  }

  public Rotation2d getVerticalAngle() {
    return verticalWrist.getAngle();
  }

  public Rotation2d getHorizontalAngle() {
    return horizontalWrist.getAngle();
  }
  public void setIntakeAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.horizontalIntakeAngle);
    verticalWrist.setAngle(Constants.CoralHandler.vertialIntakeAngle);
  }

  //TODO add a set 1st level angle method

  public void setLevelTwoAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.horizontalLevel2Angle);
    verticalWrist.setAngle(Constants.CoralHandler.verticallLevel2Angle);
  }
  
  public void setLevelThreeAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.horizontalLevel3Angle);
    verticalWrist.setAngle(Constants.CoralHandler.verticallLevel3Angle);
  }
  public void setLevelFourAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.horizontalLevel4Angle);
    verticalWrist.setAngle(Constants.CoralHandler.verticallLevel4Angle);
  }
  // //TODO Change how this works
  // public void setLevelOneAngle() {
  //   horizontalWrist.setAngle(Constants.CoralHandler.horizontalLevel1Angle);
  //   verticalWrist.setAngle(Constants.CoralHandler.verticallLevel1Angle);
  // }

  @Override
  public void periodic() {
    // Values avalible shown on SmartDashboard
    SmartDashboard.getBoolean("CoralHandler/Has Coral", false);
  }

  public Command runCoralIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.coralIntakeSpeed), this),
        Commands.waitUntil(
            () -> hasCoral()),
        Commands.runOnce(
            () -> stopOuttakeMotor(), this));
  }

  public Command runCoralOuttakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.coralOuttakeSpeed), this),
        Commands.waitUntil(
            () -> !hasCoral()),
        Commands.waitSeconds(.5),
        Commands.runOnce(
            () -> stopOuttakeMotor(), this));
  }

  public Command setVerticalAngleCommand(Rotation2d vTargetAngle) {
    return verticalWrist.setAngleCommand(vTargetAngle);
  }
  public Command setHorizontalAngleCommand(Rotation2d vTargetAngle) {
    return horizontalWrist.setAngleCommand(vTargetAngle);
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
        horizontalWrist.systemCheckCommand(),
        verticalWrist.systemCheckCommand(),

        // outtake motor system check
        Commands.runOnce( () -> runOuttakeMotor(1), this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < Constants.CoralHandler.outtakeMotorMinVelocity) {
                addFault("[System Check] Outtake Coral Motor too slow (forward direction)", false, true);
              }
            }, this),
        Commands.runOnce(
            () -> runOuttakeMotor(-1), this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < -Constants.CoralHandler.outtakeMotorMinVelocity) {
                addFault("[System Check] Outtake Coral Motor too slow (backwards direction)", false, true);
              }
            }, this));
  }
}