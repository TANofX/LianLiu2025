// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.CoralHandlerAngleEstimator;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.ManualCoralHandler;
import frc.robot.commands.Notifications;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.RobotMechanism;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final RobotMechanism robotMechanism = new RobotMechanism(() -> swerve.getPose());
  public static final Elevator elevator = new Elevator(Constants.Elevator.MOTOR_ID);
  public static final AlgaeHandler leftAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.LEFT_ALGAE_MOTOR_ID,
      Constants.AlgaeHandler.LEFT_ALGAE_SOLENOID_ID, Constants.AlgaeHandler.LEFT_ALGAE_LIMIT_ID);
  public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.RIGHT_ALGAE_MOTOR_ID,
      Constants.AlgaeHandler.RIGHT_ALGAE_SOLENOID_ID, Constants.AlgaeHandler.RIGHT_ALGAE_LIMIT_ID);
  public static final Climber climber = new Climber(Constants.Climber.CLIMBER_MOTOR_ID, Constants.Climber.PCM_ID,
      Constants.Climber.FORWARD_SOLENOID_ID, Constants.Climber.REVERSE_SOLENOID_ID, Constants.Climber.ENCODER_ID);
  public static final CoralHandler coralHandler = new CoralHandler(
      Constants.CoralHandler.OUTTAKE_MOTOR_ID,
      Constants.CoralHandler.HORIZONTAL_MOTOR_ID,
      Constants.CoralHandler.VERTICAL_MOTOR_ID,
      Constants.CoralHandler.HORIZONTAL_ENCODER_ID,
      Constants.CoralHandler.VERTICAL_ENCODER_ID);

  public RobotContainer() {
    configureButtonBindings();
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData(swerve.zeroModulesCommand());

    LEDs.setDefaultCommand(new Notifications());

    SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Left Algae Handler Test", leftAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler Test", rightAlgaeHandler.getSystemCheckCommand());

    // Climber SmartDashboard Commands
    SmartDashboard.putData("Climber/Prepare Climber", climber.getPrepareCommand());
    SmartDashboard.putData("Climber/Climb Climber", climber.climbCommand(Rotation2d.fromDegrees(-140)));
    SmartDashboard.putData("Climber/Run Claw Motor", climber.runClimberMotorCommand());
    SmartDashboard.putData("Climber/Reverse Claw Motor", climber.reverseClimbMotorCommand());
    SmartDashboard.putData("Climber/Climber System Check", climber.getSystemCheckCommand());
    SmartDashboard.putData("Climber/Calibrate Climber", climber.getCalibrateCommand(false));
    SmartDashboard.putData("Climber/Reverse Calibrate Command", climber.getCalibrateCommand(true));
    SmartDashboard.putData("Climber/Set-90", climber.setClimberNeg90());

    // Coral Handler SmartDashboard Commands
    SmartDashboard.putData("CoralHandler/Horizontal Run Positive", coralHandler.runHorizontalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Horizontal Run Negative", coralHandler.runHorizontalMotorNegativeCommand());
    SmartDashboard.putData("CoralHandler/Vertical Run Positive", coralHandler.runVerticalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Vertial Run Negative", coralHandler.runVerticalMotorNegativeCommand());
    SmartDashboard.putData("Calibrate/Zero Coral Wrist", coralHandler.zeroWristCommand());

    SmartDashboard.putData("CoralHandler/Horizontal to +45degrees",
        coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("CoralHandler/Vertical to +20degrees",
        coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(20)));
    SmartDashboard.putData("CoralHandler/Horizontal to -45degrees",
        coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(-45)));
    SmartDashboard.putData("CoralHandler/Vertical to -20degrees",
        coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-20)));
    SmartDashboard.putData("CoralHandler/Vertical to -30degrees",
        coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-30)));
    SmartDashboard.putData("CoralHandler/Horizontal to Max",
        coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(95)));
    SmartDashboard.putData("CoralHandler/Set Angles to Home", coralHandler.setHomeAngleCommand());
    SmartDashboard.putData("CoralHandler/Set Angles to Zero", coralHandler.setToZeroAngleCommand());
    SmartDashboard.putData("CoralHandler/Run Inake Wheel", coralHandler.runCoralIntakeCommand());
    SmartDashboard.putData("CoralHandler/Run Extake Wheel", coralHandler.runCoralOuttakeCommand());

    // Elevator SmartDashboard Values
    SmartDashboard.putData("Elevator/Calibrate Elevator", elevator.getCalibrationCommand());
    SmartDashboard.putData("Elevator/Check Elevator", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Elevator/Elevator 1.25", elevator.getElevatorHeightCommand(1.25));
    SmartDashboard.putData("Elevator/Elevator 0.0", elevator.getElevatorHeightCommand(0.0));
    SmartDashboard.putData("Elevator/Move Elevator UP", elevator.getSlowElevatorUpCommand());
    SmartDashboard.putData("Elevator/Move Elevator Down", elevator.getSlowElevatorDownCommand());
    SmartDashboard.putData("Elevator/Elevator 1.35", elevator.getElevatorHeightCommand(0.0));

    // Register Named Commands for pathplanner ??
    // ADD THESE COMMANDS ONCE WE DEVELOP THEM MORE:
    // NamedCommands.registerCommand("ElevatorL4",
    // elevator.getElevatorHeightCommand(0));
    // NamedCommands.registerCommand("ElevatorL1",
    // elevator.getElevatorHeightCommand(0.00000001));
    // NamedCommands.registerCommand("ElevatorIntake",
    // elevator.getElevatorHeightCommand(0.00001));
    // NamedCommands.registerCommand("Collect", new ______());

    // Register Named Commands for pathplanner examples
    // NamedCommands.registerCommand("ReadyToShootInSpeaker", new ShootInSpeaker());
    // NamedCommands.registerCommand("SpeakerShot", new Shoot(false));
    // NamedCommands.registerCommand("New AutoSpeakerShot",
    // newAutoShootInSpeaker());
    // NamedCommands.registerCommand("", );
  }

  private void configureButtonBindings() {
    elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    coralHandler.setDefaultCommand(
        new ManualCoralHandler(() -> {
          if (coDriver.DUp().getAsBoolean()) {
            return 0.5;
          }
          if (coDriver.DDown().getAsBoolean()) {
            return -0.5;
          }
          return 0.0;
        }, () -> {
          if (coDriver.DRight().getAsBoolean()) {
            return -0.5;
          }
          if (coDriver.DLeft().getAsBoolean()) {
            return 0.5;
          }
          return 0.0;
        }));

    driver.LT().whileTrue(leftAlgaeHandler.getAlgaeIntakeCommand());
    driver.LB().onTrue(leftAlgaeHandler.shootAlgaeCommand());
    driver.RT().whileTrue(rightAlgaeHandler.getAlgaeIntakeCommand());
    driver.RB().onTrue(rightAlgaeHandler.shootAlgaeCommand());
    driver.B().onTrue(climber.getPrepareCommand());
    driver.A().onTrue(climber.climbCommand(Rotation2d.fromDegrees(-150)));
    driver.Y().onTrue(climber.getPrepareCommand());

    coDriver.A().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    coDriver.Y().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(71.87-24.0)));
    coDriver.B().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(47.59-24.0)));
    coDriver.X().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(31.72-24.0)));
    coDriver.LT().whileTrue(coralHandler.intakeCommand());
    coDriver.RT().onTrue(coralHandler.runCoralOuttakeCommand());
  }

  public static void periodic() {
  }
}