// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.ManualCoralHandler;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.RobotMechanism;

import java.security.CodeSigner;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private Command autoCommand;
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);
  
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final Elevator elevator = new Elevator(Constants.Elevator.MOTOR_ID);
  public static final RobotMechanism robotMechanism = new RobotMechanism(() -> swerve.getPose());
  public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.OUTTAKE_MOTOR_ID,
      Constants.CoralHandler.HORIZONTAL_MOTOR_ID,
      Constants.CoralHandler.VERTICAL_MOTOR_ID,
      Constants.CoralHandler.HORIZONTAL_ENCODER_ID,
      Constants.CoralHandler.VERTICAL_ENCODER_ID);
  public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.RIGHT_ALGAE_MOTOR_ID,
      Constants.AlgaeHandler.RIGHT_ALGAE_SOLENOID_ID, Constants.AlgaeHandler.RIGHT_ALGAE_LIMIT_ID);
  public static final Climber climber = new Climber(Constants.Climber.CLIMBER_MOTOR_ID, Constants.Climber.PCM_ID,
      Constants.Climber.FORWARD_SOLENOID_ID, Constants.Climber.REVERSE_SOLENOID_ID, Constants.Climber.ENCODER_ID);
  public static final LEDs LEDs = new LEDs(rightAlgaeHandler, coralHandler);


  public RobotContainer() {
    configureButtonBindings();
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData(swerve.zeroModulesCommand());
    RobotContainer.swerve.setDefaultCommand(new SwerveDriveWithGamepad(() -> {
      return elevator.getHeightFrac();
    }));
    elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    // SmartDashboard.putData("Left Algae Handler Test",
    // leftAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler Test",
        rightAlgaeHandler.getSystemCheckCommand());

    // SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());
    // SmartDashboard.putData("Left Algae Handler Test", leftAlgaeHandler.getSystemCheckCommand());
    // SmartDashboard.putData("Right Algae Handler Test", rightAlgaeHandler.getSystemCheckCommand());
    //CameraServer.startAutomaticCapture().setResolution(200, 150);

    // // Climber SmartDashboard Commands
    SmartDashboard.putData("Climber/Prepare Climber", climber.getPrepareCommand());
    SmartDashboard.putData("Climber/Climb Climber", climber.climbCommand(Rotation2d.fromDegrees(-140)));
    SmartDashboard.putData("Climber/Run Claw Motor", climber.runClimberMotorCommand());
    SmartDashboard.putData("Climber/Reverse Claw Motor", climber.reverseClimbMotorCommand());
    SmartDashboard.putData("Climber/Climber System Check", climber.getSystemCheckCommand());
    SmartDashboard.putData("Climber/Calibrate Climber", climber.getCalibrateCommand(false));
    SmartDashboard.putData("Climber/Reverse Calibrate Command", climber.getCalibrateCommand(true));
    SmartDashboard.putData("Climber/Set-90", climber.setClimberNeg90());
    SmartDashboard.putData("Climber/Open Claw", climber.getOpenClawCommand());
    SmartDashboard.putData("Climber/Close Claw", climber.getCloseClawCommand());
    SmartDashboard.putData("Calibrate/Zero Coral Wrist", coralHandler.zeroWristCommand());
    SmartDashboard.putData("CoralHandler/Horizontal to +10degrees",
        coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("CoralHandler/Vertical to +10degrees",
        coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(20)));
    SmartDashboard.putData("CoralHandler/Horizontal to -10degrees",
        coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(-45)));
    SmartDashboard.putData("CoralHandler/Vertical to +-10degrees",
        coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-20)));

    registerNamedCommands();

  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

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

    SmartDashboard.putData("CoralHandler/Horizontal to +10degrees", coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("CoralHandler/Vertical to +10degrees", coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(20)));
    SmartDashboard.putData("CoralHandler/Horizontal to -10degrees", coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(-45)));
    SmartDashboard.putData("CoralHandler/Vertical to +-10degrees", coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-20)));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Place L1", level1AutoPlaceCommand());
    // NamedCommands.registerCommand("Place L2", elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    // NamedCommands.registerCommand("Place L3", elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    NamedCommands.registerCommand("Place L4", level4AutoPlaceCommand());
    // NamedCommands.registerCommand("Intake", coralHandler.runCoralIntakeCommand());
    // NamedCommands.registerCommand("Outtake", coralHandler.runCoralOuttakeCommand());
    NamedCommands.registerCommand("Collect", intakeCommand());
    NamedCommands.registerCommand("Place", Commands.none());
    NamedCommands.registerCommand(("Complete Place"), completePlaceCommand());
  }
  public void initalizeAutos() {
    autoCommand = Commands.sequence(elevator.getCalibrationCommand(), autoChooser.getSelected());

    RobotContainer.swerve.removeDefaultCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
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

    // driver.LT().whileTrue(leftAlgaeHandler.getAlgaeIntakeCommand());
    // driver.LB().onTrue(leftAlgaeHandler.shootAlgaeCommand());
    
    driver.RT().whileTrue(rightAlgaeHandler.getAlgaeIntakeCommand());
    driver.RB().onTrue(rightAlgaeHandler.shootAlgaeCommand());
    driver.A().onTrue(climber.climbCommand(Rotation2d.fromDegrees(-137)));
    driver.Y().onTrue(climber.getPrepareCommand());
    
    coDriver.A().onTrue(level1PositionCommand());
    coDriver.X().onTrue(level2PositionCommand());
    coDriver.B().onTrue(level3PositionCommand());
    coDriver.Y().onTrue(level4PositionCommand());
    coDriver.RS().onTrue(coralHandler.setHomeAngleCommand());

    coDriver.LT().whileTrue(intakeCommand());
    coDriver.RT().onTrue(coralHandler.runCoralOuttakeCommand());
    coDriver.BACK().whileTrue(coralHandler.runCoralIntakeCommand());
    coDriver.START().whileTrue(flickAlgaeCommand());
    coDriver.LB().onTrue(coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(92)));
    coDriver.RB().onTrue(coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(-86)));
  }

  public Command flickAlgaeCommand() {
    return Commands.parallel(
      coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(0)),
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(35)),
      coralHandler.runFlickSpeedCommand(0.3)
    ).finallyDo(() -> {
      coralHandler.stopOuttakeMotor();
    });
  }

  public Command intakeCommand() {
    return Commands.parallel(
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-30)),
      coralHandler.runCoralIntakeCommand(),
      elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS)
    );
  }

   public Command level1PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Units.inchesToMeters(0.5)),
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(0))
    );
  }
  public Command level2PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Units.inchesToMeters(33.72-24.0)),
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(25))
    );
  }
  
  public Command level3PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Units.inchesToMeters(51.59-24.0)),
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(25))
    );
  }
  
  public Command level4PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Units.inchesToMeters(54.0)),
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(39.7))
    );
  }

  public Command level1AutoPlaceCommand() {
    return Commands.sequence(
      level1PositionCommand(),
      coralHandler.runCoralOuttakeCommand()
    );
  }
  public Command level4AutoPlaceCommand() {
    return Commands.sequence(
      level4PositionCommand(),
      coralHandler.runCoralOuttakeCommand()
    );
  }
  public Command completePlaceCommand() {
    Rotation2d horizontalAngle = coralHandler.getHorizontalAngle();
    Rotation2d loweringAngle = horizontalAngle.minus(Rotation2d.fromDegrees(Math.signum(horizontalAngle.getDegrees())*45));
    return Commands.sequence(
      coralHandler.setHorizontalAngleCommand(loweringAngle),
      elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS),
      coralHandler.setIntakeAngleCommand()
    );
  }
  
  public void initalizeTele() {
    swerve.setDefaultCommand(new SwerveDriveWithGamepad(elevator::getHeightFrac));
  }

  public static void periodic() {
    robotMechanism.update();
  }
}