// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.ManualCoralHandler;
import frc.robot.commands.CoralHandlerAngleEstimator;
import frc.robot.commands.ElevatorJoystickControl;
import frc.robot.commands.Notifications;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.subsystems.*;
import frc.robot.util.RobotMechanism;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;




public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  

  // Subsystems
  public static final Vision vision = new Vision();
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final LEDs LEDs = new LEDs();
  public static final Elevator elevator = new Elevator(Constants.Elevator.motorCanID);
  public static final RobotMechanism robotMechanism = new RobotMechanism();
  public static final CoralHandler coralHandler = new CoralHandler(Constants.CoralHandler.outtakeMotorID, Constants.CoralHandler.horizontalMotorID, Constants.CoralHandler.verticalMotorID, Constants.CoralHandler.horizontalEncoderID, Constants.CoralHandler.verticalEncoderID);
   public static final AlgaeHandler leftAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.leftAlgaeMotorCANID, Constants.AlgaeHandler.leftAlgaeSolenoidID,Constants.AlgaeHandler.leftAlgaeLimitID);
   public static final AlgaeHandler rightAlgaeHandler = new AlgaeHandler(Constants.AlgaeHandler.rightAlgaeMotorCANID, Constants.AlgaeHandler.rightAlgaeSolenoidID, Constants.AlgaeHandler.rightAlgaeLimitID);
 
  public static final Climber climber = new Climber(Constants.Climber.MOTOR_CANID, Constants.Climber.PCMID, Constants.Climber.FORWARDSOLENOID, Constants.Climber.REVERSESOLENOID,Constants.Climber.climberEncoderCanID);

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    coralHandler.registerSystemCheckWithSmartDashboard();
    SmartDashboard.putData("Elevator Test", elevator.getSystemCheckCommand());

    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
    //elevator.setDefaultCommand(new ElevatorJoystickControl(coDriver::getLeftY));
    SmartDashboard.putData("Left Algae Handler Test", leftAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Right Algae Handler Test", rightAlgaeHandler.getSystemCheckCommand());
    SmartDashboard.putData("Raise claw", climber.runClawMotorUpCommand());
    SmartDashboard.putData("Raise and then lower claw", climber.runClawMotorOneWayThenOther());
    SmartDashboard.putData("Calibrate Climber", climber.getCalibrateCommand());
    SmartDashboard.putData("Prepare Climber", climber.getPrepareCommand());
  
  
  
    // Register Named Commands for pathplanner
    //ADD THESE COMMANDS ONCE WE DEVELOP THEM MORE:
    NamedCommands.registerCommand("ElevatorL4", elevator.getElevatorHeightCommand(0));
    NamedCommands.registerCommand("ElevatorL1", elevator.getElevatorHeightCommand(0.00000001));
    NamedCommands.registerCommand("ElevatorIntake", elevator.getElevatorHeightCommand(0.00001));
    //NamedCommands.registerCommand("Collect", new ______());
  
    
    //Do I need this?
    elevator.setDefaultCommand(new ElevatorJoystickControl(driver::getLeftY));
    coralHandler.setDefaultCommand(new ManualCoralHandler(coDriver::getLeftY, coDriver::getLeftX));
    // SmartDashboard.putData(intake.getIntakePivotTuner());
    // SmartDashboard.putData(intake.getIntakeTuner());
    //SmartDashboard.putData("Tune Elevation", shooterWrist.getElevationTunerCommand());
    //SmartDashboard.putData("Tune Shooter", shooter.getShooterTunerCommand());
    //SmartDashboard.putData("Tune Shooter Intake", shooter.getIntakeTunerCommand());
    //SmartDashboard.putData("Tune Intake", intake.getIntakeTuner());
    // SmartDashboard.putData(Commands.runOnce(() -> {
    // intake.updateRotationOffset();}, intake));

    //SmartDashboard.putData("Tune Elevator Motor", elevator.getHeightTunerCommand());
    //SmartDashboard.putData("Elevator Extents", new FindMotorExtents())
    // SmartDashboard.putData("Robot At Center Blue Ring", Commands.runOnce(() -> {
    //   swerve.resetOdometry(new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0)));
    // }, swerve));
    // SmartDashboard.putData("Robot At Red Speaker", new AtRedSubWoofer());

    // Register Named Commands for pathplanner
    //NamedCommands.registerCommand("ReadyToShootInSpeaker", new ShootInSpeaker());
    //NamedCommands.registerCommand("SpeakerShot", new Shoot(false));
    //NamedCommands.registerCommand("New AutoSpeakerShot", newAutoShootInSpeaker());
    // NamedCommands.registerCommand("", );
    
    //PPHolonomicDriveController.setRotationTargetOverride(this::overrideAngle);
  }
  
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

    SmartDashboard.putData("Calibrate/Zero Coral Wrist", coralHandler.zeroWristCommand());
   
    driver.LT().onTrue(leftAlgaeHandler.getAlgaeIntakeCommand());
    driver.LB().onTrue(leftAlgaeHandler.shootAlgaeCommand());
    driver.RT().onTrue(rightAlgaeHandler.getAlgaeIntakeCommand());
    driver.RB().onTrue(rightAlgaeHandler.shootAlgaeCommand());
    coDriver.A().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MIN_HEIGHT_METERS));
    coDriver.B().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(20.0)));
    coDriver.Y().onTrue(elevator.getElevatorHeightCommand(Units.inchesToMeters(40.0)));
    coDriver.X().onTrue(elevator.getElevatorHeightCommand(Constants.Elevator.MAX_HEIGHT_METERS));
    driver.Y().onTrue(climber.getPrepareCommand());
    driver.A().onTrue(climber.getCalibrateCommand());
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
  /*   
        }, shooter))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          shooter.stopMotors();

        }))))); */
    //driver.B().onTrue(climber.getStowCommand());
    //driver.Y().onTrue(climber.getPrepareCommand());
    //driver.X().onTrue(climber.getCloseCommand());
    //driver.A().onTrue(climber.getClimbCommand());
   

  }


  public static void periodic() {
    robotMechanism.update();
  }
}