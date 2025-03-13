// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.signals.SensorDirectionValue;
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

  private int isIntaking = 0;

  // Creation of Flywheel Simulation for the simulation of the outtakeMotor
  private final FlywheelSim coralHandlerOuttakePhysicsSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 
                                          Constants.CoralHandler.OUTTAKE_JKMETERS_SQUARED, 
                                          Constants.CoralHandler.OUTTAKE_MOTOR_GEARING),
      DCMotor.getNeoVortex(1), 
      Constants.CoralHandler.OUTTAKE_MOTOR_GEARING);

  public CoralHandler(int outtakeMotorID, int horizontalMotorID, int verticalMotorID, int horizontalAbsoluteEncoderID,
      int verticalAbsoluteEncoderID) {
    super("CoralHandler");
    // Creation of Motors, Encoders, and Limitswitch for CoralHandler Subsystem
    outtakeMotor = new SparkFlex(outtakeMotorID, MotorType.kBrushless);

    horizontalWrist = new CoralHandlerWrist(
            "Horizontal",
            horizontalMotorID,
            horizontalAbsoluteEncoderID,
            Constants.CoralHandler.HORIZONTAL_GEAR_RATIO,
            Constants.CoralHandler.HORIZONTAL_MOTOR_POS_P,
            Constants.CoralHandler.HORIZONTAL_MOTOR_POS_I,
            Constants.CoralHandler.HORIZONTAL_MOTOR_POS_D,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_POS_P, 
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_POS_I, 
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_POS_D, 
            Constants.CoralHandler.HORIZONTAL_MOTOR_POS_FEED_FORWARD,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_POS_FEED_FORWARD,
            Constants.CoralHandler.HORIZONTAL_MOTOR_POS_I_ZONE,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_POS_I_ZONE,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MIN_VELOCITY,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_VELOCITY,
            Constants.CoralHandler.HORIZONTAL_MOTOR_MAX_ACCELERATION,
            Constants.CoralHandler.HORIZONTAL_MOTOR_CLOSED_LOPP_ERROR,
            Type.kNormallyOpen,
            Constants.CoralHandler.HORIZONTAL_MIN_ANGLE,
            Constants.CoralHandler.HORIZONTAL_MAX_ANGLE,
            Constants.CoralHandler.HORIZONTAL_JKMETERS_SQUARED,
            Constants.CoralHandler.CORAL_END_EFFECTOR_LENGTH,
            Constants.CoralHandler.HORIZONTAL_STARTING_ANGLE_IN_RADIANS,
            false,
            SensorDirectionValue.Clockwise_Positive,
            Constants.CoralHandler.HORIZONTAL_SOFT_LIMIT_FORWARD_ANGLE,
            Constants.CoralHandler.HORIZONTAL_SOFT_LIMIT_REVERSE_ANGLE,
            Constants.CoralHandler.HORIZONTAL_ROTATION_DEGREES_PER_ROTATION
            );
    horizontalWrist.registerSystemCheckWithSmartDashboard();
    verticalWrist = new CoralHandlerWrist(
            "Vertical",
            verticalMotorID,
            verticalAbsoluteEncoderID,
            Constants.CoralHandler.VERTICAL_GEAR_RATIO,
            Constants.CoralHandler.VERTICAL_MOTOR_POS_P,
            Constants.CoralHandler.VERTICAL_MOTOR_POS_I,
            Constants.CoralHandler.VERTICAL_MOTOR_POS_D,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_POS_P,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_POS_I, 
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_POS_D, 
            Constants.CoralHandler.VERTICAL_MOTOR_POS_FEED_FORWARD,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_POS_FEED_FORWARD, 
            Constants.CoralHandler.VERTICAL_MOTOR_POS_I_ZONE,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_POS_I_ZONE,
            Constants.CoralHandler.VERTICAL_MOTOR_MIN_VELOCITY,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_VELOCITY,
            Constants.CoralHandler.VERTICAL_MOTOR_MAX_ACCELERATION,
            Constants.CoralHandler.VERTICAL_MOTOR_CLOSED_LOOP_ERROR,
            Type.kNormallyOpen,
            Constants.CoralHandler.VERTICAL_MIN_ANGLE,
            Constants.CoralHandler.VERTICAL_MIN_ANGLE,
            Constants.CoralHandler.VERTICAL_JKMETERS_SQUARED,
            Constants.CoralHandler.CORAL_END_EFFECTOR_LENGTH,
            Constants.CoralHandler.VERTICAL_STARTING_ANGLE_IN_RADIANS,
            true,
            SensorDirectionValue.Clockwise_Positive,
            Constants.CoralHandler.VERTICAL_SOFT_LIMIT_FORWARD_ANGLE,
            Constants.CoralHandler.VERTICAL_SOFT_LIMIT_REVERSE_ANGLE,
            Constants.CoralHandler.VERTICAL_ROTATION_DEGREES_PER_ROTATION
    );


    verticalWrist.registerSystemCheckWithSmartDashboard();

    outtakeEncoder = outtakeMotor.getEncoder();
    // TODO Forward or reverse limit switch?
    coralLimitSwitch = outtakeMotor.getForwardLimitSwitch(); 
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
        / Constants.CoralHandler.OUTTAKE_MOTOR_GEARING;
    

    // Creation of the motor simulations
    coralHandlerOuttakeSim.iterate(outtakeMotorVelocity, RobotController.getBatteryVoltage(), 0.02);

    // Creation of the absolute encoder simulations

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(coralHandlerOuttakeSim.getMotorCurrent(),
        horizontalWrist.motorSim.getMotorCurrent(), verticalWrist.motorSim.getMotorCurrent()));
  }

  /**
   * Stops motor for the coral end effector intake/outtake motor. Sets motor speed
   * to zero.
   */
  public int getIntaking(){
    return isIntaking;
  }

  public void stopOuttakeMotor() {
    outtakeMotor.stopMotor();
    isIntaking = 0;
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
    isIntaking = 0;
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
    if(outtakeMotorSpeed > 0){
      isIntaking = 1;
    }else if(outtakeMotorSpeed < 0){
      isIntaking = -1;
    }else{
      isIntaking = 0;
    }
  }
  
  public void runSuckMotor() {
    outtakeMotor.set(Constants.CoralHandler.CORAL_OUTTAKE_SPEED);
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
   * Sets the vertical positioning of the coral end effector to a specified angle.
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
    horizontalWrist.setAngle(Constants.CoralHandler.HORIZONTAL_INTAKE_ANGLE);
    verticalWrist.setAngle(Constants.CoralHandler.VERTICAL_INTAKE_ANGLE);
  }

   // //TODO Change how this works using auto adjustments
  // public void setLevelOneAngle() {
  //   horizontalWrist.setAngle(Constants.CoralHandler.horizontalLevel1Angle);
  //   verticalWrist.setAngle(Constants.CoralHandler.verticalLevel1Angle);
  // }

  public void setLevelTwoAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.HORIZONTAL_LEVEL_2_ANGLE);
    verticalWrist.setAngle(Constants.CoralHandler.VERTICAL_LEVEL_2_ANGLE);
  }

  public void setLevelThreeAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.HORIZONTAL_LEVEL_3_ANGLE);
    verticalWrist.setAngle(Constants.CoralHandler.VERTICAL_LEVEL_3_ANGLE);
  }
  public void setLevelFourAngle() {
    horizontalWrist.setAngle(Constants.CoralHandler.HORIZONTAL_LEVEL_4_ANGLE);
    verticalWrist.setAngle(Constants.CoralHandler.VERTICAL_LEVEL_4_ANGLE);
  }
 
  @Override
  public void periodic() {
    // Values available shown on SmartDashboard
    SmartDashboard.getBoolean("CoralHandler/Has Coral", false);
    SmartDashboard.putNumber("CoralHandler/Horizontal Wrist Relative Angle (Deg.)", horizontalWrist.getAngle().getDegrees());
    SmartDashboard.putNumber("CoralHandler/Vertical Wrist Relative Angle (Deg.)", verticalWrist.getAngle().getDegrees());
    SmartDashboard.putNumber("CoralHandler/Horizontal Wrist Absolute Angle (Deg.)", horizontalWrist.getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber("CoralHandler/Vertical Wrist Absolute Angle (Deg.)", verticalWrist.getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber("CoralHandler/Horizontal Wrist Motor Position (Rot.)", horizontalWrist.getMotorRotations().getRotations());
    SmartDashboard.putNumber("CoralHandler/Vertical Wrist Motor Position (Rot.)", verticalWrist.getMotorRotations().getRotations());
    SmartDashboard.putNumber("CoralHandler/Horizontal Applied Output", horizontalWrist.getAppliedOutput());
    SmartDashboard.putNumber("CoralHandler/Vertical Applied Output", verticalWrist.getAppliedOutput());
  }

  public Command holdCoralCommand() {
    return Commands.run(
      () -> {
      runOuttakeMotor(-.1);});
  }

  public Command holdRightCommand() {
    return Commands.sequence(
    setVerticalAngleCommand(Rotation2d.fromDegrees(38)),  
    setHorizontalAngleCommand(Rotation2d.fromDegrees(-80))
    );
  }
  
  public Command zeroWristCommand() {
  return Commands.runOnce(
          () -> {
              horizontalWrist.updateWristOffset();
              verticalWrist.updateWristOffset();
          }, this)
          .ignoringDisable(true);
  }

  public Command runFlickSpeedCommand(double speed) {
    return Commands.run(
      () -> {
        outtakeMotor.set(speed);
      }, this);
  }

  public Command runCoralIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.CORAL_INTAKE_SPEED), this),
        Commands.waitUntil(
            () -> hasCoral())).finallyDo( () -> {
              stopOuttakeMotor();
            }
            );
  }

  public Command runCoralOuttakeCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> runOuttakeMotor(Constants.CoralHandler.CORAL_OUTTAKE_SPEED), this),
        Commands.waitSeconds(.5),
        Commands.waitUntil(
            () -> !hasCoral()),
        Commands.runOnce(
            () -> 
            stopOuttakeMotor()
            , this), 
            verticalWrist.setAngleCommand(Rotation2d.fromDegrees(-20)));
  }

  public Command setToZeroAngleCommand() {
    return Commands.parallel(
        verticalWrist.setAngleCommand(Rotation2d.fromDegrees(0)),
        horizontalWrist.setAngleCommand(Rotation2d.fromDegrees(0))
    );
  }

  public Command setHomeAngleCommand() {
    return Commands.parallel(
      verticalWrist.setAngleCommand(Rotation2d.fromDegrees(82)),
      horizontalWrist.setAngleCommand(Rotation2d.fromDegrees(92))
    );
  }

  public Command setIntakeAngleCommand() {
    return Commands.parallel(
      verticalWrist.setAngleCommand(Rotation2d.fromDegrees(-30)),
      horizontalWrist.setAngleCommand(Rotation2d.fromDegrees(92))
    );
  }

  public Command setVerticalAngleCommand(Rotation2d vTargetAngle) {
    return verticalWrist.setAngleCommand(vTargetAngle);
  }
  public Command setHorizontalAngleCommand(Rotation2d vTargetAngle) {
    return horizontalWrist.setAngleCommand(vTargetAngle);
  }
  
  public Command runHorizontalMotorPositiveCommand() {
    return Commands.run(() -> {horizontalWrist.runCoralWrist(0.05);}, this).finallyDo(() -> horizontalWrist.stopMotor());
  }

  public Command runHorizontalMotorNegativeCommand() {
    return Commands.run(() -> {horizontalWrist.runCoralWrist(-0.05);}, this).finallyDo(() -> horizontalWrist.stopMotor());
  }

  public Command runVerticalMotorPositiveCommand() {
    return Commands.run(() -> {verticalWrist.runCoralWrist(0.1);}, this).finallyDo(() -> verticalWrist.stopMotor());
  }

  public Command runVerticalMotorNegativeCommand() {
    return Commands.run(() -> {verticalWrist.runCoralWrist(-0.1);}, this).finallyDo(() -> verticalWrist.stopMotor());
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
              if ((outtakeEncoder.getVelocity()) < Constants.CoralHandler.OUTTAKE_MOTOR_MIN_VELOCITY) {
                addFault("[System Check] Outtake Coral Motor too slow (forward direction)", false, true);
              }
            }, this),
        Commands.runOnce(
            () -> runOuttakeMotor(-1), this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if ((outtakeEncoder.getVelocity()) < -Constants.CoralHandler.OUTTAKE_MOTOR_MIN_VELOCITY) {
                addFault("[System Check] Outtake Coral Motor too slow (backwards direction)", false, true);
              }
            }, this));
  }

  public boolean isHorizontalAtSetpoint() {
    return horizontalWrist.isAtTargetAngle();
  }

  public boolean isVerticalAtSetpoint() {
    return verticalWrist.isAtTargetAngle();
  }
}