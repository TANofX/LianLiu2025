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

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);



  // Subsystems
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final Elevator elevator = new Elevator(Constants.Elevator.MOTOR_ID);
  public static final RobotMechanism robotMechanism = new RobotMechanism(() -> swerve.getPose());
  public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.OUTTAKE_MOTOR_ID,
  Constants.CoralHandler.HORIZONTAL_MOTOR_ID,
  Constants.CoralHandler.VERTICAL_MOTOR_ID,
  Constants.CoralHandler.HORIZONTAL_ENCODER_ID,
  Constants.CoralHandler.VERTICAL_ENCODER_ID);
  // public static final AlgaeHandler leftAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.LEFT_ALGAE_MOTOR_ID,
  //     Constants.AlgaeHandler.LEFT_ALGAE_SOLENOID_ID, Constants.AlgaeHandler.LEFT_ALGAE_LIMIT_ID);
  public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.RIGHT_ALGAE_MOTOR_ID,
      Constants.AlgaeHandler.RIGHT_ALGAE_SOLENOID_ID, Constants.AlgaeHandler.RIGHT_ALGAE_LIMIT_ID);

  public static final Climber climber = new Climber(Constants.Climber.CLIMBER_MOTOR_ID, Constants.Climber.PCM_ID,
      Constants.Climber.FORWARD_SOLENOID_ID, Constants.Climber.REVERSE_SOLENOID_ID, Constants.Climber.ENCODER_ID);

  public RobotContainer() {
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());

    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
    elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    // SmartDashboard.putData("Left Algae Handler Test",
    // leftAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler Test",
    rightAlgaeHandler.getSystemCheckCommand());

    SmartDashboard.putData("Prepare Climber", climber.getPrepareCommand());
    SmartDashboard.putData("Climb Climber", climber.climbCommand(Rotation2d.fromDegrees(-140)));
    SmartDashboard.putData("Run Claw Motor", climber.runClimberMotorCommand());
    SmartDashboard.putData("Reverse Claw Motor", climber.reverseClimbMotorCommand());
    SmartDashboard.putData("Climber System Check", climber.getSystemCheckCommand());
    SmartDashboard.putData("Calibrate Climber", climber.getCalibrateCommand(false));
    SmartDashboard.putData("Reverse Calibrate Command", climber.getCalibrateCommand(true));
    SmartDashboard.putData("Climber/Set-90", climber.setClimberNeg90());


    SmartDashboard.putData("Calibrate/Zero Coral Wrist", coralHandler.zeroWristCommand());
    SmartDashboard.putData("CoralHandler/Horizontal to +10degrees", coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(45)));
    SmartDashboard.putData("CoralHandler/Vertical to +10degrees", coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(20)));
    SmartDashboard.putData("CoralHandler/Horizontal to -10degrees", coralHandler.setHorizontalAngleCommand(Rotation2d.fromDegrees(-45)));
    SmartDashboard.putData("CoralHandler/Vertical to +-10degrees", coralHandler.setVerticalAngleCommand(Rotation2d.fromDegrees(-20)));

// Register Named Commands for pathplanner
    NamedCommands.registerCommand("Place L1", elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    NamedCommands.registerCommand("Place L2", elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    NamedCommands.registerCommand("Place L3", elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    NamedCommands.registerCommand("Place L4", elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    NamedCommands.registerCommand("Intake", coralHandler.runCoralIntakeCommand());
    NamedCommands.registerCommand("Outtake", coralHandler.runCoralOuttakeCommand());

  }
    
    //PPHolonomicDriveController.setRotationTargetOverride(this::overrideAngle);
  

  private void configureButtonBindings() {

    coDriver.START();
    coDriver.RT().onTrue(new CoralHandlerAngleEstimator());
    // coralHandler.setDefaultCommand(new ManualCoralHandler(coDriver::getLeftY, coDriver::getLeftX));
    coralHandler.setDefaultCommand(new ManualCoralHandler(() -> {
      if (coDriver.DUp().getAsBoolean()) {
        return 0.5;
      }
      if (coDriver.DDown().getAsBoolean()){
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
    coDriver.A().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    coDriver.B().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    coDriver.Y().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(40.0)));
    coDriver.X().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MAX_HEIGHT_METERS));
    driver.B().onTrue(climber.getPrepareCommand());
    driver.A().onTrue(climber.climbCommand(Rotation2d.fromDegrees(-150)));

    coDriver.START();
    driver.Y().onTrue(climber.getPrepareCommand());
    coDriver.START();
    coDriver.RT().onTrue(new CoralHandlerAngleEstimator());



//

   //ONCE WE ADD ALGAE TO MAIN THESE COMMANDS SHOULD WORK:
   //driver.LT().onTrue(new getAlgaeIntakeCommand());
   //driver.RT().onTrue(new shootAlgaeCommand());
   //driver.START().onTrue(new ); //callibrate elevator
    coDriver.START();
    SmartDashboard.putData("Calibrate Elevator", elevator.getCalibrationCommand());
    SmartDashboard.putData("Check Elevator", elevator.getSystemCheckCommand());
    SmartDashboard.putData("Elevator 1.25", elevator.getElevatorHeightCommand(1.25));
    SmartDashboard.putData("Elevator 0.0", elevator.getElevatorHeightCommand(0.0));
    SmartDashboard.putData("Move Elevator UP", elevator.getSlowElevatorUpCommand());
    SmartDashboard.putData("Move Elevator Down", elevator.getSlowElevatorDownCommand());
    SmartDashboard.putData("Elevator 1.35", elevator.getElevatorHeightCommand(0.0));
    SmartDashboard.putData("CoralHandler/Horizontal Run Positive", coralHandler.runHorizontalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Horizontal Run Negative", coralHandler.runHorizontalMotorNegativeCommand());
    SmartDashboard.putData("CoralHandler/Vertical Run Positive", coralHandler.runVerticalMotorPositiveCommand());
    SmartDashboard.putData("CoralHandler/Vertial Run Negative", coralHandler.runVerticalMotorNegativeCommand());
     
    /*  
  
  
        }, shooter))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          shooter.stopMotors();

        }))))); */
    //driver.B().onTrue(climber.getStowCommand());
    //driver.Y().onTrue(climber.getPrepareCommand());
    //driver.X().onTrue(climber.getCloseCommand());
    //driver.A().onTrue(climber.getClimbCommand());


    // ONCE WE ADD ALGAE TO MAIN THESE COMMANDS SHOULD WORK:
    // driver.LT().onTrue(new getAlgaeIntakeCommand());
    // driver.RT().onTrue(new shootAlgaeCommand());
    // driver.START().onTrue(new ); //calibrate elevator

    coDriver.START();
  }

  public static void periodic() {
  }
}