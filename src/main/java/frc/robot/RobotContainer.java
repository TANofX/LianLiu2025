// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.AutoAimCoralHandler;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.ManualCoralHandler;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.AutoAiming;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ReefTargeting;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ReefTargeting.BranchPosition;
import frc.robot.util.RobotMechanism;

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
  public static final ReefTargeting reefTargeting = new ReefTargeting(() -> swerve.getPose());
  public static final AutoAiming autoAimer = new AutoAiming(() -> robotMechanism.getFieldPositionOfCoralHandler(), () -> reefTargeting.getBranchTargetPose2d());

  

  public RobotContainer() {
    configureButtonBindings();
    registerNamedCommands();
    
    coralHandler.registerSystemCheckWithSmartDashboard();
    elevator.registerSystemCheckWithSmartDashboard();
    rightAlgaeHandler.registerSystemCheckWithSmartDashboard();
    climber.registerSystemCheckWithSmartDashboard();
    swerve.registerSystemCheckWithSmartDashboard();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    
    SmartDashboard.putData(swerve.zeroModulesCommand());
    RobotContainer.swerve.setDefaultCommand(new SwerveDriveWithGamepad(() -> {
      return elevator.getHeightFrac();
    }));
    elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));

    //System Check Commands
    SmartDashboard.putData("Elevator System Check", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler System Check", rightAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("CoralHandler System Check", coralHandler.getSystemCheckCommand());
    SmartDashboard.putData("Swerve System Check", swerve.getSystemCheckCommand());

    //Climber SmartDashboard Commands
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
    SmartDashboard.putData("Climber/Stow Position", climber.climbCommand(Rotation2d.fromDegrees(-137)));

    //Coral Handler SmartDashboard Commands
    SmartDashboard.putData("CoralHandler/Zero Coral Wrist", coralHandler.zeroWristCommand());
    SmartDashboard.putData("CoralHandler/Horizontal Run Positive", coralHandler.runHorizontalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Horizontal Run Negative", coralHandler.runHorizontalMotorNegativeCommand());
    SmartDashboard.putData("CoralHandler/Vertical Run Positive", coralHandler.runVerticalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Vertial Run Negative", coralHandler.runVerticalMotorNegativeCommand());
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
    SmartDashboard.putData("CoralHandler/Run Motor Positive (Intake/Outtake Motor)", coralHandler.runOuttakeMotorCommand(0.75));
    SmartDashboard.putData("CoralHandler/Run Motor Negative (Intake/Outtake Motor)", coralHandler.runOuttakeMotorCommand(-0.75));

    // Elevator SmartDashboard Values
    SmartDashboard.putData("Elevator/Calibrate Elevator", elevator.getCalibrationCommand());
    SmartDashboard.putData("Elevator/Check Elevator", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Elevator/Elevator 1.25", elevator.getElevatorHeightCommand(1.25));
    SmartDashboard.putData("Elevator/Elevator 0.0", elevator.getElevatorHeightCommand(0.0));
    SmartDashboard.putData("Elevator/Move Elevator UP", elevator.getSlowElevatorUpCommand());
    SmartDashboard.putData("Elevator/Move Elevator Down", elevator.getSlowElevatorDownCommand());
    SmartDashboard.putData("Elevator/Elevator 1.35", elevator.getElevatorHeightCommand(0.0));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Place L1", level1AutoPlaceCommand());
    NamedCommands.registerCommand("Place L4", level4AutoPlaceCommand());
    NamedCommands.registerCommand("Collect", intakeCommand());
    NamedCommands.registerCommand("Complete Place", completePlaceCommand());
    NamedCommands.registerCommand("HoldCoral", coralHandler.holdCoralCommand());
    NamedCommands.registerCommand("Level Prep", levelPrepCommand());
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
            return -0.375; //-0.75;
          }
          if (coDriver.DDown().getAsBoolean()) {
            return 0.375; //0.75;
          }
          return 0.0;
        }, () -> {
          if (coDriver.DRight().getAsBoolean()) {
            return -0.375; //0.75;
          }
          if (coDriver.DLeft().getAsBoolean()) {
            return 0.375; //0.75;
          }
          return 0.0;
        }));
    
    driver.RT().whileTrue(rightAlgaeHandler.getAlgaeIntakeCommand());
    driver.RB().onTrue(rightAlgaeHandler.shootAlgaeCommand());
    driver.LB().onTrue(climbCommand());
    driver.LT().onTrue(climber.getPrepareCommand());
    driver.X().onTrue(Commands.runOnce(
      () -> {
        reefTargeting.setTargetAprilTag(BranchPosition.LEFT);
        Optional<Pose2d> targetPose = reefTargeting.getRobotTargetPose2d();
        if(targetPose.isPresent())
          swerve.goToPoseCommand(targetPose.get(), targetPose.get().getRotation().plus(Rotation2d.fromDegrees(90.0))).schedule();
      }
    ));
    driver.B().onTrue(Commands.runOnce(
      () -> {
        reefTargeting.setTargetAprilTag(BranchPosition.RIGHT);
        Optional<Pose2d> targetPose = reefTargeting.getRobotTargetPose2d();
        if(targetPose.isPresent())
          swerve.goToPoseCommand(targetPose.get(), targetPose.get().getRotation().plus(Rotation2d.fromDegrees(90.0))).schedule();
      }
    ));

    coDriver.A().onTrue(level1PositionCommand());
    coDriver.X().onTrue(level2PositionCommand());
    coDriver.B().onTrue(level3PositionCommand());
    coDriver.Y().onTrue(level4PositionCommand());
    coDriver.RS().onTrue(coralHandler.setHomeAngleCommand());

    coDriver.LT().whileTrue(intakeCommand());
    coDriver.RT().onTrue(coralHandler.runCoralOuttakeCommand());
    coDriver.BACK().whileTrue(coralHandler.runCoralIntakeCommand());
    coDriver.START().whileTrue(flickAlgaeCommand());
    coDriver.LB().onTrue(coralHandler.setHorizontalAngleCommand(Constants.CoralHandler.HORIZONTAL_MAX_LEFT_ANGLE));
    coDriver.RB().onTrue(coralHandler.setHorizontalAngleCommand(Constants.CoralHandler.HORIZONTAL_MAX_RIGHT_ANGLE));
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
      coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-35)),
      coralHandler.runCoralIntakeCommand(),
      elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS),
      coralHandler.determineDirectionCommand()
    );
  }   

   public Command level1PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Constants.Elevator.LEVEL1_HEIGHT),
      coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_LEVEL1_ANGLE)
    );
  }
  public Command level2PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Constants.Elevator.LEVEL2_HEIGHT),
      coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_LEVEL2_ANGLE)
    );
  }
  
  public Command level3PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Constants.Elevator.LEVEL3_HEIGHT),
      coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_LEVEL3_ANGLE)
    );
  }
  
  public Command level4PositionCommand() {
    return Commands.parallel(
      elevator.getElevatorHeightCommand(Constants.Elevator.LEVEL4_HEIGHT),
      coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_LEVEL4_ANGLE)
    );
  }

  public Command climbCommand() {
    return Commands.parallel(
      climber.climbCommand(Rotation2d.fromDegrees(-137)),
      coralHandler.setToZeroAngleCommand()
    );
  }

  public Command level1AutoPlaceCommand() {
    return Commands.sequence(
      level1PositionCommand(),
      coralHandler.runCoralOuttakeCommand()
    );
  }

  public Command levelPrepCommand() {
    return coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_AUTO_PREP_ANGLE);
  }

  public Command level4AutoPlaceCommand() {
    return Commands.sequence(
      elevator.getElevatorHeightCommand(Constants.Elevator.LEVEL4_HEIGHT),
      coralHandler.setVerticalAngleCommand(Constants.CoralHandler.VERTICAL_LEVEL4_ANGLE),
      coralHandler.setHorizontalAngleCommand(Constants.CoralHandler.HORIZONTAL_MAX_LEFT_ANGLE),
      Commands.waitSeconds(0.2),
      coralHandler.runCoralOuttakeCommand()
    );
  }

  public Command completePlaceCommand() {
    Rotation2d horizontalAngle = coralHandler.getHorizontalAngle();
    Rotation2d loweringAngle = horizontalAngle.minus(Rotation2d.fromDegrees(Math.signum(horizontalAngle.getDegrees())*20));
    return Commands.sequence(
      Commands.parallel(
        coralHandler.setHorizontalAngleCommand(loweringAngle),
        elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS)
      )
    );
  }
  
  public void initalizeTele() {
    swerve.setDefaultCommand(new SwerveDriveWithGamepad(elevator::getHeightFrac));
  }

  public static void periodic() {
    robotMechanism.update();
  }
} 